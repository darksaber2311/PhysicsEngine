// test_sandbox_controls.cpp -- Unit tests for Milestone 9 Sandbox Controls logic.
//
// Verification Scope:
//   - Simulation pause / resume transitions
//   - Single-stepping while paused (advances exactly 1 step, remains paused)
//   - Deterministic reset behavior (restores initial scene state identically)
//   - Runtime gravity modification with immediate velocity integration effect
//   - Body spawn descriptor validation and boundary clamping
//   - Broad-phase algorithm runtime switching (BruteForce, SpatialGrid, SpatialGridParallel)
//   - Engine body capacity limit enforcement and graceful rejection

#include "PhysicsEngine/Concurrency/WorkerPool.h"
#include "PhysicsEngine/Math/Vec2.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Physics/Collision.h"
#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/SimulationController.h"
#include "PhysicsEngine/Physics/WorldConfig.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {

Engine createTestEngine(uint32_t maxBodies = 64, Vec2 gravity = {0.f, -9.81f}, float fixedDt = 1.f / 120.f)
{
    WorldConfig cfg;
    cfg.maxBodies = maxBodies;
    cfg.gravity = gravity;
    cfg.fixedDt = fixedDt;
    return Engine{cfg};
}

void populateStandardScene(Engine& engine)
{
    engine.clear();

    // Static floor
    BodyDesc floor;
    floor.position    = Vec2{0.f, -5.0f};
    floor.mass        = 0.f;
    floor.shapeType   = ShapeType::AABB;
    floor.radius      = 10.0f;
    floor.halfHeight  = 0.5f;
    floor.restitution = 0.5f;
    (void)engine.addBody(floor);

    // Dynamic circle
    BodyDesc circle;
    circle.position    = Vec2{0.f, 2.0f};
    circle.mass        = 1.0f;
    circle.shapeType   = ShapeType::Circle;
    circle.radius      = 0.5f;
    circle.restitution = 0.7f;
    (void)engine.addBody(circle);

    // Dynamic box
    BodyDesc box;
    box.position    = Vec2{1.5f, 4.0f};
    box.mass        = 2.0f;
    box.shapeType   = ShapeType::AABB;
    box.radius      = 0.6f;
    box.halfHeight  = 0.6f;
    box.restitution = 0.4f;
    (void)engine.addBody(box);
}

} // anonymous namespace

TEST_CASE("Sandbox Controls: Pause and resume transitions", "[sandbox][controls]")
{
    Engine engine = createTestEngine();
    SimulationController controller;

    BodyDesc desc;
    desc.position = Vec2{0.f, 10.f};
    desc.mass     = 1.0f;
    desc.radius   = 0.5f;
    BodyHandle h = engine.addBody(desc);
    REQUIRE(engine.isValidHandle(h));

    const Vec2 initialPos = engine.getPosition(h);

    SECTION("Paused state halts advance()") {
        controller.pause();
        REQUIRE(controller.isPaused());

        const uint32_t steps = controller.advance(engine, 0.1f);
        CHECK(steps == 0);
        CHECK(engine.getPosition(h).x == Approx(initialPos.x));
        CHECK(engine.getPosition(h).y == Approx(initialPos.y));
        CHECK(controller.accumulator() == 0.f);
    }

    SECTION("Resume re-enables advance()") {
        controller.pause();
        controller.advance(engine, 0.1f);
        controller.resume();
        REQUIRE_FALSE(controller.isPaused());

        const uint32_t steps = controller.advance(engine, 1.f / 60.f);
        CHECK(steps == 2); // 1/60s with 1/120s fixedDt = 2 steps
        CHECK(engine.getPosition(h).y < initialPos.y); // dynamic body fell under gravity
    }
}

TEST_CASE("Sandbox Controls: Single-step while paused", "[sandbox][controls]")
{
    Engine engine = createTestEngine();
    SimulationController controller;

    BodyDesc desc;
    desc.position = Vec2{0.f, 10.f};
    desc.mass     = 1.0f;
    desc.radius   = 0.5f;
    BodyHandle h = engine.addBody(desc);
    REQUIRE(engine.isValidHandle(h));

    controller.pause();
    REQUIRE(controller.isPaused());

    const float fixedDt = engine.config().fixedDt;
    const Vec2 initPos = engine.getPosition(h);
    const Vec2 initVel = engine.getVelocity(h);

    // Execute exactly one single step
    controller.singleStep(engine);

    // Controller must remain paused
    CHECK(controller.isPaused());

    // Velocity integrated by gravity * fixedDt: v1 = v0 + g * dt
    const Vec2 expVel = initVel + engine.config().gravity * fixedDt;
    CHECK(engine.getVelocity(h).x == Approx(expVel.x));
    CHECK(engine.getVelocity(h).y == Approx(expVel.y));

    // Position integrated: p1 = p0 + v1 * dt
    const Vec2 expPos = initPos + expVel * fixedDt;
    CHECK(engine.getPosition(h).x == Approx(expPos.x));
    CHECK(engine.getPosition(h).y == Approx(expPos.y));

    // A subsequent single-step advances exactly one more step
    controller.singleStep(engine);
    CHECK(controller.isPaused());

    const Vec2 expVel2 = expVel + engine.config().gravity * fixedDt;
    CHECK(engine.getVelocity(h).y == Approx(expVel2.y));
}

TEST_CASE("Sandbox Controls: Deterministic scene reset", "[sandbox][controls]")
{
    Engine engine = createTestEngine();
    SimulationController controller;

    populateStandardScene(engine);
    REQUIRE(engine.bodyCount() == 3);

    // Record initial states
    std::vector<Vec2> initialPositions;
    std::vector<Vec2> initialVelocities;
    for (uint32_t i = 0; i < engine.bodyCount(); ++i) {
        initialPositions.push_back(engine.positionsData()[i]);
        initialVelocities.push_back(engine.velocitiesData()[i]);
    }

    // Advance simulation by 1.0 second (120 steps), causing movement and contacts
    for (int step = 0; step < 120; ++step) {
        controller.singleStep(engine);
    }

    // Verify bodies have moved
    CHECK(engine.positionsData()[1].y < initialPositions[1].y);

    // Reset scene
    populateStandardScene(engine);
    REQUIRE(engine.bodyCount() == 3);

    // Verify all positions and velocities match initial state exactly
    for (uint32_t i = 0; i < engine.bodyCount(); ++i) {
        CHECK(engine.positionsData()[i].x == initialPositions[i].x);
        CHECK(engine.positionsData()[i].y == initialPositions[i].y);
        CHECK(engine.velocitiesData()[i].x == initialVelocities[i].x);
        CHECK(engine.velocitiesData()[i].y == initialVelocities[i].y);
    }
}

TEST_CASE("Sandbox Controls: Runtime gravity editing", "[sandbox][controls]")
{
    Engine engine = createTestEngine();
    SimulationController controller;

    BodyDesc desc;
    desc.position = Vec2{0.f, 0.f};
    desc.velocity = Vec2{0.f, 0.f};
    desc.mass     = 1.0f;
    desc.radius   = 0.5f;
    BodyHandle h = engine.addBody(desc);
    REQUIRE(engine.isValidHandle(h));

    const float dt = engine.config().fixedDt;

    SECTION("Zero-G preset") {
        engine.setGravity(Vec2{0.f, 0.f});
        controller.singleStep(engine);

        CHECK(engine.getVelocity(h).x == Approx(0.f));
        CHECK(engine.getVelocity(h).y == Approx(0.f));
    }

    SECTION("Horizontal gravity (wind)") {
        engine.setGravity(Vec2{10.f, 0.f});
        controller.singleStep(engine);

        CHECK(engine.getVelocity(h).x == Approx(10.f * dt));
        CHECK(engine.getVelocity(h).y == Approx(0.f));
    }

    SECTION("Inverted gravity (upward)") {
        engine.setGravity(Vec2{0.f, 9.81f});
        controller.singleStep(engine);

        CHECK(engine.getVelocity(h).y == Approx(9.81f * dt));
        CHECK(engine.getPosition(h).y > 0.f);
    }
}

TEST_CASE("Sandbox Controls: Spawn parameter validation and clamping", "[sandbox][controls]")
{
    Engine engine = createTestEngine();

    // Clamping helper mirroring UI validation logic
    auto clampAndValidate = [](BodyDesc& d, bool isStatic) {
        d.radius = std::max(0.05f, d.radius);
        d.halfHeight = std::max(0.05f, d.halfHeight);
        d.restitution = std::clamp(d.restitution, 0.0f, 1.0f);
        d.mass = isStatic ? 0.0f : std::max(0.01f, d.mass);
    };

    SECTION("Circle with non-positive radius is clamped to safe minimum") {
        BodyDesc d;
        d.shapeType = ShapeType::Circle;
        d.radius = -2.5f;
        d.mass = 1.0f;
        d.restitution = 1.5f; // above 1.0
        clampAndValidate(d, false);

        CHECK(d.radius >= 0.05f);
        CHECK(d.restitution == 1.0f);

        BodyHandle h = engine.addBody(d);
        REQUIRE(engine.isValidHandle(h));
        CHECK(engine.getShapeType(h) == ShapeType::Circle);
        CHECK(engine.getRadius(h) >= 0.05f);
    }

    SECTION("Static body has inverse mass zero regardless of input mass") {
        BodyDesc d;
        d.shapeType = ShapeType::AABB;
        d.radius = 1.0f;
        d.halfHeight = 1.0f;
        d.mass = 50.0f;
        clampAndValidate(d, true);

        CHECK(d.mass == 0.0f);

        BodyHandle h = engine.addBody(d);
        REQUIRE(engine.isValidHandle(h));
        CHECK(engine.getInverseMass(h) == 0.0f);
    }

    SECTION("Initial velocity is assigned correctly") {
        BodyDesc d;
        d.shapeType = ShapeType::Circle;
        d.radius = 0.5f;
        d.mass = 1.0f;
        d.velocity = Vec2{12.5f, -8.0f};
        clampAndValidate(d, false);

        BodyHandle h = engine.addBody(d);
        REQUIRE(engine.isValidHandle(h));
        CHECK(engine.getVelocity(h).x == Approx(12.5f));
        CHECK(engine.getVelocity(h).y == Approx(-8.0f));
    }
}

TEST_CASE("Sandbox Controls: Broad-phase runtime switching", "[sandbox][controls]")
{
    Engine engine = createTestEngine();
    SimulationController controller;
    WorkerPool pool{2};
    controller.setWorkerPool(&pool);

    // Two overlapping circles to generate a contact
    BodyDesc c1;
    c1.position  = Vec2{0.f, 0.f};
    c1.mass      = 1.0f;
    c1.radius    = 1.0f;
    BodyHandle h1 = engine.addBody(c1);

    BodyDesc c2;
    c2.position  = Vec2{1.5f, 0.f};
    c2.mass      = 1.0f;
    c2.radius    = 1.0f;
    BodyHandle h2 = engine.addBody(c2);

    REQUIRE(engine.isValidHandle(h1));
    REQUIRE(engine.isValidHandle(h2));

    // 1. Brute Force
    controller.setBroadPhaseType(BroadPhaseType::BruteForce);
    CHECK(controller.broadPhaseType() == BroadPhaseType::BruteForce);
    controller.singleStep(engine);
    REQUIRE(controller.lastContacts().size() == 1);
    const float penBF = controller.lastContacts()[0].penetration;

    // Reset positions
    engine.setPosition(h1, Vec2{0.f, 0.f});
    engine.setPosition(h2, Vec2{1.5f, 0.f});

    // 2. Spatial Grid Sequential
    controller.setBroadPhaseType(BroadPhaseType::SpatialGrid);
    CHECK(controller.broadPhaseType() == BroadPhaseType::SpatialGrid);
    controller.singleStep(engine);
    REQUIRE(controller.lastContacts().size() == 1);
    const float penGrid = controller.lastContacts()[0].penetration;
    CHECK(penGrid == Approx(penBF));

    // Reset positions
    engine.setPosition(h1, Vec2{0.f, 0.f});
    engine.setPosition(h2, Vec2{1.5f, 0.f});

    // 3. Spatial Grid Parallel
    controller.setBroadPhaseType(BroadPhaseType::SpatialGridParallel);
    CHECK(controller.broadPhaseType() == BroadPhaseType::SpatialGridParallel);
    controller.singleStep(engine);
    REQUIRE(controller.lastContacts().size() == 1);
    const float penPar = controller.lastContacts()[0].penetration;
    CHECK(penPar == Approx(penBF));
}

TEST_CASE("Sandbox Controls: Capacity limit enforcement", "[sandbox][controls]")
{
    constexpr uint32_t kMax = 4;
    Engine engine = createTestEngine(kMax);

    BodyDesc d;
    d.radius = 0.5f;
    d.mass   = 1.0f;

    // Add up to capacity
    for (uint32_t i = 0; i < kMax; ++i) {
        d.position = Vec2{static_cast<float>(i) * 2.0f, 0.f};
        BodyHandle h = engine.addBody(d);
        CHECK(engine.isValidHandle(h));
    }
    CHECK(engine.bodyCount() == kMax);

    // Attempt to exceed capacity
    d.position = Vec2{100.f, 0.f};
    BodyHandle overflowHandle = engine.addBody(d);

    // Must return null handle safely without crash or reallocation
    CHECK_FALSE(engine.isValidHandle(overflowHandle));
    CHECK(overflowHandle.isNull());
    CHECK(overflowHandle == BodyHandle::null());
    CHECK(engine.bodyCount() == kMax);
}
