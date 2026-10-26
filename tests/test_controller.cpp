// test_controller.cpp -- Catch2 tests for SimulationController.
//
// Coverage:
//   - Construction and default configuration
//   - Frame time shorter than fixedDt (no step, accumulator increases)
//   - Frame time producing exactly one step
//   - Frame time producing multiple steps
//   - Accumulator behavior across multiple small frames
//   - Catch-up step limits (spiral-of-death prevention and excess time discard)
//   - Pause and resume behavior
//   - Explicit single-step execution while paused
//   - Runtime gravity changes via controller update loop
//   - Force application immediately before a step
//   - Correct force clearing after consumption across substeps
//   - Invalid elapsed-time inputs (zero, negative, NaN, infinity)
//   - Accumulator reset
//   - SubStepAlpha interpolation calculation
//   - Headless deterministic execution (no OpenGL, no sleeps)

#include "PhysicsEngine/Physics/SimulationController.h"
#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/WorldConfig.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Math/Vec2.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <limits>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {

Engine makeTestEngine(float fixedDt = 1.f / 120.f, Vec2 gravity = {0.f, -9.81f})
{
    WorldConfig cfg;
    cfg.fixedDt = fixedDt;
    cfg.gravity = gravity;
    cfg.maxBodies = 32;
    return Engine{cfg};
}

BodyHandle addDynamicBody(Engine& e, Vec2 pos = {0.f, 0.f}, Vec2 vel = {0.f, 0.f}, float mass = 1.f)
{
    BodyDesc d;
    d.position = pos;
    d.velocity = vel;
    d.mass     = mass;
    d.radius   = 1.f;
    return e.addBody(d);
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Construction and Defaults
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SimulationController: default construction", "[controller][ctor]")
{
    SimulationController controller;
    REQUIRE(controller.maxSubsteps() == SimulationController::kDefaultMaxSubsteps);
    REQUIRE(controller.accumulator() == 0.f);
    REQUIRE_FALSE(controller.isPaused());
}

TEST_CASE("SimulationController: custom maxSubsteps clamped to at least 1", "[controller][ctor]")
{
    SimulationController c0(0);
    REQUIRE(c0.maxSubsteps() == 1);

    SimulationController c5(5);
    REQUIRE(c5.maxSubsteps() == 5);

    c5.setMaxSubsteps(0);
    REQUIRE(c5.maxSubsteps() == 1);

    c5.setMaxSubsteps(10);
    REQUIRE(c5.maxSubsteps() == 10);
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame Time Processing
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SimulationController: frame time shorter than fixedDt does not step", "[controller][advance]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt);
    BodyHandle h = addDynamicBody(engine, {0.f, 10.f});

    SimulationController controller;

    // Advance by half a physics step
    uint32_t steps = controller.advance(engine, dt * 0.5f);

    REQUIRE(steps == 0);
    REQUIRE(controller.accumulator() == Approx(dt * 0.5f));
    // Physics has not advanced
    REQUIRE(engine.getPosition(h).y == Approx(10.f));
    REQUIRE(engine.getVelocity(h).y == 0.f);
}

TEST_CASE("SimulationController: frame time producing exactly one step", "[controller][advance]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt);
    BodyHandle h = addDynamicBody(engine, {0.f, 10.f});

    SimulationController controller;

    uint32_t steps = controller.advance(engine, dt);

    REQUIRE(steps == 1);
    REQUIRE(controller.accumulator() == Approx(0.f).margin(1e-7f));
    // Dynamic body has fallen by one semi-implicit Euler step
    // vy = 0 + g * dt = -9.81 * (1/120)
    // y = 10 + vy * dt
    const float expected_vy = -9.81f * dt;
    const float expected_y  = 10.f + expected_vy * dt;
    REQUIRE(engine.getVelocity(h).y == Approx(expected_vy).margin(1e-5f));
    REQUIRE(engine.getPosition(h).y == Approx(expected_y).margin(1e-5f));
}

TEST_CASE("SimulationController: frame time producing multiple steps", "[controller][advance]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt);
    BodyHandle h = addDynamicBody(engine, {0.f, 10.f});

    SimulationController controller;

    // Exactly 3 steps with 0 remainder
    uint32_t steps = controller.advance(engine, 3.f * dt);

    REQUIRE(steps == 3);
    REQUIRE(controller.accumulator() == Approx(0.f).margin(1e-7f));

    // Expected discrete velocity after 3 steps
    const float expected_vy = -9.81f * 3.f * dt;
    REQUIRE(engine.getVelocity(h).y == Approx(expected_vy).margin(1e-4f));
}

TEST_CASE("SimulationController: accumulator behavior across multiple small frames", "[controller][accumulator]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt);
    BodyHandle h = addDynamicBody(engine, {0.f, 10.f});

    SimulationController controller;

    // Frame 1: 0.4 * dt -> 0 steps, acc = 0.4 * dt
    REQUIRE(controller.advance(engine, 0.4f * dt) == 0);
    REQUIRE(controller.accumulator() == Approx(0.4f * dt).margin(1e-7f));

    // Frame 2: 0.4 * dt -> 0 steps, acc = 0.8 * dt
    REQUIRE(controller.advance(engine, 0.4f * dt) == 0);
    REQUIRE(controller.accumulator() == Approx(0.8f * dt).margin(1e-7f));

    // Frame 3: 0.4 * dt -> total 1.2 * dt -> 1 step, acc = 0.2 * dt
    REQUIRE(controller.advance(engine, 0.4f * dt) == 1);
    REQUIRE(controller.accumulator() == Approx(0.2f * dt).margin(1e-7f));

    // Body stepped exactly once
    const float expected_vy = -9.81f * dt;
    REQUIRE(engine.getVelocity(h).y == Approx(expected_vy).margin(1e-5f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Catch-Up Limit & Spiral-of-Death Prevention
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SimulationController: catch-up step limit clamps substeps and discards excess time", "[controller][catchup]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt);

    SimulationController controller(4); // max 4 substeps

    // Simulate a massive 1.0-second stall (120 dt equivalent)
    uint32_t steps = controller.advance(engine, 1.0f);

    // Must be clamped to maxSubsteps
    REQUIRE(steps == 4);

    // Excess accumulated time must be discarded to prevent spiral-of-death
    REQUIRE(controller.accumulator() == 0.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Pause, Resume, and Single Step
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SimulationController: pause prevents advance() from stepping and accumulating", "[controller][pause]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt);
    BodyHandle h = addDynamicBody(engine, {0.f, 10.f});

    SimulationController controller;
    controller.pause();
    REQUIRE(controller.isPaused());

    // Advance while paused
    uint32_t steps = controller.advance(engine, 2.f * dt);

    REQUIRE(steps == 0);
    REQUIRE(controller.accumulator() == 0.f); // does not accumulate time while paused
    REQUIRE(engine.getPosition(h).y == Approx(10.f));
    REQUIRE(engine.getVelocity(h).y == 0.f);

    // Resume
    controller.resume();
    REQUIRE_FALSE(controller.isPaused());

    // Now advances normally
    steps = controller.advance(engine, dt);
    REQUIRE(steps == 1);
    REQUIRE(engine.getVelocity(h).y < 0.f);
}

TEST_CASE("SimulationController: explicit single-step execution works while paused", "[controller][pause][step]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt);
    BodyHandle h = addDynamicBody(engine, {0.f, 10.f});

    SimulationController controller;
    controller.pause();

    // Call singleStep() directly
    controller.singleStep(engine);

    // One physics step executed
    const float expected_vy = -9.81f * dt;
    REQUIRE(engine.getVelocity(h).y == Approx(expected_vy).margin(1e-5f));
    // Accumulator unaffected
    REQUIRE(controller.accumulator() == 0.f);

    // Another single step
    controller.step(engine);
    const float expected_vy2 = -9.81f * 2.f * dt;
    REQUIRE(engine.getVelocity(h).y == Approx(expected_vy2).margin(1e-5f));
}

TEST_CASE("SimulationController: resetAccumulator clears unsimulated time", "[controller][reset]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt);

    SimulationController controller;
    controller.advance(engine, dt * 0.75f);
    REQUIRE(controller.accumulator() == Approx(dt * 0.75f));

    controller.resetAccumulator();
    REQUIRE(controller.accumulator() == 0.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Force Lifecycle via Controller
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SimulationController: force applied immediately before advance() is consumed and cleared", "[controller][force]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt, {0.f, 0.f}); // zero gravity
    BodyHandle h = addDynamicBody(engine, {0.f, 0.f}, {0.f, 0.f}, 1.f);

    SimulationController controller;

    // Apply force immediately before advance()
    engine.addForce(h, {12.f, 0.f});

    // Advance exactly 1 step
    uint32_t steps = controller.advance(engine, dt);
    REQUIRE(steps == 1);

    // Force was consumed: vx = 12 * dt
    const float expected_vx = 12.f * dt;
    REQUIRE(engine.getVelocity(h).x == Approx(expected_vx).margin(1e-5f));

    // Force buffer must be cleared
    REQUIRE(engine.forcesData()[0].x == 0.f);

    // Next step without new forces: velocity remains constant (no duplicate force)
    steps = controller.advance(engine, dt);
    REQUIRE(steps == 1);
    REQUIRE(engine.getVelocity(h).x == Approx(expected_vx).margin(1e-5f));
}

TEST_CASE("SimulationController: multi-step frame consumes force on first step only", "[controller][force]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt, {0.f, 0.f}); // zero gravity
    BodyHandle h = addDynamicBody(engine, {0.f, 0.f}, {0.f, 0.f}, 1.f);

    SimulationController controller;

    // Apply an impulse force of 10 N
    engine.addForce(h, {10.f, 0.f});

    // Advance 2 steps in a single frameTime (2 * dt)
    uint32_t steps = controller.advance(engine, 2.f * dt);
    REQUIRE(steps == 2);

    // Force should only have been consumed on step 1 (cleared immediately after).
    // So vx = 10 * dt, NOT 20 * dt!
    const float expected_vx = 10.f * dt;
    REQUIRE(engine.getVelocity(h).x == Approx(expected_vx).margin(1e-5f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Runtime Gravity Changes via Controller Loop
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SimulationController: runtime gravity change takes effect in next advance()", "[controller][gravity]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt, {0.f, -10.f});
    BodyHandle h = addDynamicBody(engine, {0.f, 0.f});

    SimulationController controller;

    // Step 1: gravity = -10
    controller.advance(engine, dt);
    REQUIRE(engine.getVelocity(h).y == Approx(-10.f * dt).margin(1e-5f));

    // Change gravity at runtime
    engine.setGravity({0.f, +20.f});

    // Step 2: gravity = +20
    controller.advance(engine, dt);
    // vy = (-10 * dt) + (20 * dt) = +10 * dt
    REQUIRE(engine.getVelocity(h).y == Approx(10.f * dt).margin(1e-5f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Input Validation
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SimulationController: invalid frameTime values are rejected", "[controller][validation]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine(dt);

    SimulationController controller;

    // Zero
    REQUIRE(controller.advance(engine, 0.f) == 0);
    REQUIRE(controller.accumulator() == 0.f);

    // Negative
    REQUIRE(controller.advance(engine, -0.016f) == 0);
    REQUIRE(controller.accumulator() == 0.f);

    // NaN
    REQUIRE(controller.advance(engine, std::numeric_limits<float>::quiet_NaN()) == 0);
    REQUIRE(controller.accumulator() == 0.f);

    // Infinity
    REQUIRE(controller.advance(engine, std::numeric_limits<float>::infinity()) == 0);
    REQUIRE(controller.accumulator() == 0.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// SubStep Alpha
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SimulationController: subStepAlpha computes correct ratio", "[controller][alpha]")
{
    const float dt = 1.f / 100.f;
    Engine engine = makeTestEngine(dt);

    SimulationController controller;

    REQUIRE(controller.subStepAlpha(engine) == 0.f);

    // Advance 0.25 * dt
    controller.advance(engine, 0.25f * dt);
    REQUIRE(controller.subStepAlpha(engine) == Approx(0.25f));

    // Advance another 0.50 * dt -> total 0.75 * dt
    controller.advance(engine, 0.50f * dt);
    REQUIRE(controller.subStepAlpha(engine) == Approx(0.75f));
}
