// test_stability.cpp -- Catch2 stability and edge-case verification for Milestone 5.
//
// Scenarios tested:
//   A. Circle falling onto a static floor:
//      - Circle does not pass through floor.
//      - Position and velocity remain finite over hundreds of steps.
//      - Quantitative settling / micro-bounce velocity recording.
//   B. Stack of several bodies:
//      - Multiple bodies stacked above static floor with overlapping initial states.
//      - Multi-thousand step simulation (3000 steps).
//      - Verification of no NaNs/inf, bounded velocities, no collapse through floor.
//   C. High-speed collision & tunneling documentation:
//      - Moderate high-speed collision (e.g. 30 m/s) collides and bounces cleanly.
//      - Extreme speed (1200 m/s) tunneling occurs due to discrete timestep (no CCD).
//   D. Solver configuration robustness & invalid value rejection:
//      - Negative, NaN, infinite slop/percent/maxCorrection sanitized gracefully.
//      - Static-static contacts safely ignored without division by zero.
//      - Positional correction divides by total inverse mass and leaves static bodies unmoved.

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

Engine makeWorld(Vec2 gravity = {0.f, -9.81f}, float fixedDt = 1.f / 120.f)
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
// Scenario A: Circle falling onto a static floor
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Stability: Scenario A - Circle falling onto static floor does not penetrate", "[stability][scenario_a]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeWorld({0.f, -9.81f}, dt);

    // Static floor: center at (0, -1), halfHeight = 1 -> top surface at y = 0
    (void)engine.addBody(makeAABB({0.f, -1.f}, 20.f, 1.f, 0.f, {0.f, 0.f}, 0.5f));

    // Dynamic circle: radius = 1.0, drop from y = 5.0
    BodyHandle circle = engine.addBody(makeCircle({0.f, 5.f}, 1.f, 1.f, {0.f, 0.f}, 0.5f));

    SimulationController controller;

    // Simulate for 600 fixed steps (5.0 seconds of physics)
    for (int step = 0; step < 600; ++step) {
        controller.step(engine);

        Vec2 pos = engine.getPosition(circle);
        Vec2 vel = engine.getVelocity(circle);

        // Verification 1: State must always be finite
        REQUIRE(std::isfinite(pos.x));
        REQUIRE(std::isfinite(pos.y));
        REQUIRE(std::isfinite(vel.x));
        REQUIRE(std::isfinite(vel.y));

        // Verification 2: Circle must NEVER fall through the floor (top surface y = 0, so y >= 0.8)
        REQUIRE(pos.y >= 0.85f);
    }

    Vec2 finalPos = engine.getPosition(circle);
    Vec2 finalVel = engine.getVelocity(circle);

    // Resting position: radius is 1.0, floor surface is 0.0 -> center settles near y = 1.0
    REQUIRE(finalPos.y == Approx(1.0f).margin(0.05f));
    REQUIRE(finalPos.x == Approx(0.0f).margin(1e-4f));

    // In discrete collision response without restitution velocity-threshold damping,
    // gravity adds |g|*dt ≈ 0.0818 m/s every step, which is repeatedly resolved.
    // Verify that velocity magnitude is bounded by single-step gravity acceleration:
    const float gStep = 9.81f * dt;
    REQUIRE(std::abs(finalVel.y) <= Approx(gStep * 2.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Scenario B: Stack of several bodies
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Stability: Scenario B - Multi-body vertical stack over 3000 steps", "[stability][scenario_b]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeWorld({0.f, -9.81f}, dt);

    // Static floor at y = -1, halfHeight = 1 -> top surface at y = 0
    (void)engine.addBody(makeAABB({0.f, -1.f}, 20.f, 1.f, 0.f, {0.f, 0.f}, 0.2f));

    // Stack 3 circles, each radius 0.5 (diameters 1.0)
    // Intentionally initialize with small initial overlaps to stress positional correction
    BodyHandle c1 = engine.addBody(makeCircle({0.f, 0.5f}, 0.5f, 1.f, {0.f, 0.f}, 0.2f));
    BodyHandle c2 = engine.addBody(makeCircle({0.f, 1.45f}, 0.5f, 1.f, {0.f, 0.f}, 0.2f)); // overlaps c1 by 0.05
    BodyHandle c3 = engine.addBody(makeCircle({0.f, 2.40f}, 0.5f, 1.f, {0.f, 0.f}, 0.2f)); // overlaps c2 by 0.05

    SimulationController controller;

    // Simulate for 3000 steps (25 seconds of simulation)
    for (int step = 0; step < 3000; ++step) {
        controller.step(engine);
    }

    Vec2 p1 = engine.getPosition(c1);
    Vec2 p2 = engine.getPosition(c2);
    Vec2 p3 = engine.getPosition(c3);
    Vec2 v1 = engine.getVelocity(c1);
    Vec2 v2 = engine.getVelocity(c2);
    Vec2 v3 = engine.getVelocity(c3);

    // Verification 1: All values strictly finite
    REQUIRE(std::isfinite(p1.x)); REQUIRE(std::isfinite(p1.y));
    REQUIRE(std::isfinite(p2.x)); REQUIRE(std::isfinite(p2.y));
    REQUIRE(std::isfinite(p3.x)); REQUIRE(std::isfinite(p3.y));
    REQUIRE(std::isfinite(v1.x)); REQUIRE(std::isfinite(v1.y));
    REQUIRE(std::isfinite(v2.x)); REQUIRE(std::isfinite(v2.y));
    REQUIRE(std::isfinite(v3.x)); REQUIRE(std::isfinite(v3.y));

    // Verification 2: Floor containment (none passed through y = 0)
    REQUIRE(p1.y >= 0.40f);
    REQUIRE(p2.y >= 1.20f);
    REQUIRE(p3.y >= 2.00f);

    // Verification 3: Relative ordering maintained (c1 at bottom, c2 middle, c3 top)
    REQUIRE(p1.y < p2.y);
    REQUIRE(p2.y < p3.y);

    // Verification 4: No unbounded velocities or explosion
    REQUIRE(std::abs(v1.y) < 1.0f);
    REQUIRE(std::abs(v2.y) < 1.0f);
    REQUIRE(std::abs(v3.y) < 1.0f);

    // Verification 5: Horizontal drift is minimal (stable column)
    REQUIRE(std::abs(p1.x) < 0.05f);
    REQUIRE(std::abs(p2.x) < 0.05f);
    REQUIRE(std::abs(p3.x) < 0.05f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Scenario C: High-speed collision & Tunneling
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Stability: Scenario C - Moderate high-speed collision bounces cleanly", "[stability][scenario_c]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeWorld({0.f, 0.f}, dt);

    // Static wall at x = 5.0, halfWidth = 0.5 -> left face at x = 4.5
    (void)engine.addBody(makeAABB({5.f, 0.f}, 0.5f, 10.f, 0.f, {0.f, 0.f}, 1.0f));

    // Dynamic circle at x = 0, radius = 0.5. Launched toward wall at 30 m/s
    // Step size dx = 30 * (1/120) = 0.25 m, which is smaller than wall thickness (1.0 m)
    BodyHandle circle = engine.addBody(makeCircle({0.f, 0.f}, 0.5f, 1.f, {30.f, 0.f}, 1.0f));

    SimulationController controller;

    // Advance 0.5 seconds (60 steps)
    uint32_t steps = 0;
    for (int i = 0; i < 60; ++i) {
        steps += controller.advance(engine, dt);
    }
    REQUIRE(steps == 60);

    Vec2 pos = engine.getPosition(circle);
    Vec2 vel = engine.getVelocity(circle);

    // Circle should have collided with wall and bounced to the left (vel.x < 0)
    REQUIRE(vel.x == Approx(-30.f));
    REQUIRE(pos.x <= 4.05f); // Did not pass through wall
}

TEST_CASE("Stability: Scenario C - Extreme speed tunneling documented as discrete collision limitation", "[stability][scenario_c]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeWorld({0.f, 0.f}, dt);

    // Static wall at x = 5.0, halfWidth = 0.5 -> left face x = 4.5, right face x = 5.5 (thickness 1.0 m)
    (void)engine.addBody(makeAABB({5.f, 0.f}, 0.5f, 10.f, 0.f, {0.f, 0.f}, 1.0f));

    // Dynamic circle at x = 0, radius = 0.5. Launched at 1200 m/s
    // In one single step, dx = 1200 * (1/120) = 10.0 m!
    // The circle jumps from x = 0 to x = 10 in a single timestep, completely skipping over the wall [4.0, 6.0]
    BodyHandle circle = engine.addBody(makeCircle({0.f, 0.f}, 0.5f, 1.f, {1200.f, 0.f}, 1.0f));

    SimulationController controller;

    // Single step
    controller.step(engine);

    Vec2 pos = engine.getPosition(circle);

    // Confirms tunneling occurred due to discrete integration without Continuous Collision Detection (CCD)
    REQUIRE(pos.x >= 9.5f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Scenario D: Solver configuration and invalid values
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Stability: Scenario D - Solver sanitizes negative, NaN, and infinite config values safely", "[stability][scenario_d]")
{
    Engine engine = makeWorld({0.f, 0.f});
    BodyHandle hA = engine.addBody(makeCircle({-0.5f, 0.f}, 1.f, 1.f, {1.f, 0.f}));
    BodyHandle hB = engine.addBody(makeCircle({0.5f, 0.f}, 1.f, 1.f, {-1.f, 0.f}));

    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.size() == 1);

    const float nanF = std::numeric_limits<float>::quiet_NaN();
    const float infF = std::numeric_limits<float>::infinity();

    // Config 1: Negative slop and NaN percent
    {
        SolverConfig badCfg;
        badCfg.slop = -1.f;
        badCfg.percent = nanF;
        badCfg.maxCorrection = -5.f;

        solveCollisions(engine, contacts, badCfg);

        REQUIRE(std::isfinite(engine.getPosition(hA).x));
        REQUIRE(std::isfinite(engine.getPosition(hB).x));
        REQUIRE(std::isfinite(engine.getVelocity(hA).x));
        REQUIRE(std::isfinite(engine.getVelocity(hB).x));
    }

    // Config 2: Infinite slop and infinite maxCorrection
    {
        SolverConfig badCfg2;
        badCfg2.slop = infF;
        badCfg2.percent = infF;
        badCfg2.maxCorrection = infF;

        solveCollisions(engine, contacts, badCfg2);

        REQUIRE(std::isfinite(engine.getPosition(hA).x));
        REQUIRE(std::isfinite(engine.getPosition(hB).x));
        REQUIRE(std::isfinite(engine.getVelocity(hA).x));
        REQUIRE(std::isfinite(engine.getVelocity(hB).x));
    }
}

TEST_CASE("Stability: Scenario D - Static-static contact zero total inverse mass skipped safely", "[stability][scenario_d]")
{
    Engine engine = makeWorld({0.f, 0.f});
    BodyHandle s1 = engine.addBody(makeCircle({0.f, 0.f}, 1.f, 0.f));
    BodyHandle s2 = engine.addBody(makeCircle({0.5f, 0.f}, 1.f, 0.f));

    Contact c;
    c.bodyA = s1;
    c.bodyB = s2;
    c.normal = {1.f, 0.f};
    c.penetration = 1.5f;

    std::vector<Contact> contacts{c};

    // Neither body should move or divide by zero
    solveCollisions(engine, contacts);

    REQUIRE(engine.getPosition(s1).x == 0.f);
    REQUIRE(engine.getPosition(s2).x == 0.5f);
    REQUIRE(engine.getVelocity(s1).x == 0.f);
    REQUIRE(engine.getVelocity(s2).x == 0.f);
}

TEST_CASE("Stability: Scenario D - Positional correction divides by total inverse mass and leaves static body unmoved", "[stability][scenario_d]")
{
    Engine engine = makeWorld({0.f, 0.f});
    // Static body A (mass = 0, invMass = 0) at origin
    BodyHandle sA = engine.addBody(makeCircle({0.f, 0.f}, 1.f, 0.f));
    // Dynamic body B (mass = 2, invMass = 0.5) overlapping body A by penetration = 0.5
    BodyHandle dB = engine.addBody(makeCircle({1.5f, 0.f}, 1.f, 2.f));

    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].penetration == Approx(0.5f));

    SolverConfig cfg;
    cfg.slop = 0.01f;
    cfg.percent = 0.5f;

    resolvePositionalCorrections(engine, contacts, cfg);

    // Static body A must NOT move at all
    REQUIRE(engine.getPosition(sA).x == 0.f);
    REQUIRE(engine.getPosition(sA).y == 0.f);

    // Dynamic body B should receive 100% of the displacement
    // depth = 0.5 - 0.01 = 0.49
    // rawMag = (0.49 / 0.5) * 0.5 = 0.49
    // posB.x = 1.5 + rawMag * invMassB = 1.5 + 0.49 * 0.5 = 1.745
    REQUIRE(engine.getPosition(dB).x == Approx(1.745f).margin(1e-4f));
}
