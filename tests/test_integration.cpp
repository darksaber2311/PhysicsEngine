// test_integration.cpp -- Catch2 tests for semi-implicit Euler integration.
//
// Strategy: test physical correctness against closed-form analytic solutions.
//   - Projectile under gravity: position and velocity match kinematic equations.
//   - Static body immunity: static bodies are not moved by integration.
//   - Zero external force: only gravity acts on dynamic body.
//   - External force: acceleration from addForce() propagates correctly.
//   - Energy check (qualitative): over many steps a falling body's kinetic
//     energy matches expected values from work-energy theorem.
//   - Timestep validation: dt <= 0 and non-finite dt are no-ops.
//   - Force clearing semantics: accumulated forces are not carried over
//     automatically; clearForces() must be called.
//   - Zero gravity config: body with initial velocity moves in a straight line.
//
// Tolerances: integration error accumulates as O(dt) per step for semi-implicit
//   Euler.  Tests use dt = 1/120 s which is fine for N <= 1000 steps.
//   Absolute tolerance is scaled to dt * N.

#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/Integration.h"
#include "PhysicsEngine/Physics/WorldConfig.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Math/Vec2.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>

using namespace PhysicsEngine;
using Catch::Approx;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static Engine makeEngine(Vec2 gravity = {0.f, -9.81f}, float dt = 1.f / 120.f)
{
    WorldConfig cfg;
    cfg.gravity  = gravity;
    cfg.fixedDt  = dt;
    return Engine{cfg};
}

static BodyHandle addDynamic(Engine& e, Vec2 pos, Vec2 vel = {},
                             float mass = 1.f, float radius = 0.5f)
{
    BodyDesc d;
    d.position  = pos;
    d.velocity  = vel;
    d.mass      = mass;
    d.radius    = radius;
    return e.addBody(d);
}

static BodyHandle addStatic(Engine& e, Vec2 pos)
{
    BodyDesc d;
    d.position = pos;
    d.mass     = 0.f;   // static
    d.radius   = 0.5f;
    return e.addBody(d);
}

// ─────────────────────────────────────────────────────────────────────────────
// Timestep validation
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Integration: zero dt is a no-op", "[integration][validation]")
{
    Engine e = makeEngine();
    BodyHandle h = addDynamic(e, {0.f, 10.f}, {0.f, 0.f});
    Vec2 posBefore = e.getPosition(h);
    Vec2 velBefore = e.getVelocity(h);

    integrate(e, 0.f);

    REQUIRE(e.getPosition(h).x == posBefore.x);
    REQUIRE(e.getPosition(h).y == posBefore.y);
    REQUIRE(e.getVelocity(h).x == velBefore.x);
    REQUIRE(e.getVelocity(h).y == velBefore.y);
}

TEST_CASE("Integration: negative dt is a no-op", "[integration][validation]")
{
    Engine e = makeEngine();
    BodyHandle h = addDynamic(e, {0.f, 10.f}, {1.f, 0.f});
    Vec2 posBefore = e.getPosition(h);

    integrate(e, -0.1f);

    REQUIRE(e.getPosition(h).y == Approx(posBefore.y));
    REQUIRE(e.getVelocity(h).x == Approx(1.f));  // unchanged
}

TEST_CASE("Integration: infinite dt is a no-op", "[integration][validation]")
{
    Engine e = makeEngine();
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {1.f, 0.f});

    integrate(e, std::numeric_limits<float>::infinity());

    REQUIRE(std::isfinite(e.getPosition(h).x));
    REQUIRE(std::isfinite(e.getVelocity(h).x));
}

TEST_CASE("Integration: NaN dt is a no-op", "[integration][validation]")
{
    Engine e = makeEngine();
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {1.f, 0.f});

    integrate(e, std::numeric_limits<float>::quiet_NaN());

    REQUIRE(std::isfinite(e.getPosition(h).x));
    REQUIRE(std::isfinite(e.getVelocity(h).x));
}

// ─────────────────────────────────────────────────────────────────────────────
// Static body immunity
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Integration: static body is not moved by gravity", "[integration][static]")
{
    Engine e = makeEngine({0.f, -9.81f});
    BodyHandle hs = addStatic(e, {5.f, 5.f});

    for (int i = 0; i < 100; ++i) {
        integrate(e, 1.f / 120.f);
    }

    REQUIRE(e.getPosition(hs).x == Approx(5.f));
    REQUIRE(e.getPosition(hs).y == Approx(5.f));
    REQUIRE(e.getVelocity(hs).x == 0.f);
    REQUIRE(e.getVelocity(hs).y == 0.f);
}

TEST_CASE("Integration: static body ignores external addForce", "[integration][static]")
{
    Engine e = makeEngine({0.f, 0.f});   // no gravity — isolate force test
    BodyHandle hs = addStatic(e, {0.f, 0.f});

    // addForce is already documented to no-op on static bodies,
    // but verify integration also skips them.
    e.addForce(hs, {1000.f, 1000.f});
    integrate(e, 1.f / 60.f);

    REQUIRE(e.getPosition(hs) == Vec2{});
    REQUIRE(e.getVelocity(hs) == Vec2{});
}

TEST_CASE("Integration: mix of static and dynamic - static unaffected", "[integration][static]")
{
    Engine e = makeEngine({0.f, -9.81f});
    BodyHandle hd = addDynamic(e, {0.f, 10.f});
    BodyHandle hs = addStatic(e, {5.f, 5.f});

    for (int i = 0; i < 60; ++i) {
        integrate(e, 1.f / 60.f);
    }

    // Dynamic body fell; static body did not move
    REQUIRE(e.getPosition(hd).y < 10.f);    // fell
    REQUIRE(e.getPosition(hs).x == Approx(5.f));
    REQUIRE(e.getPosition(hs).y == Approx(5.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Projectile under gravity — analytic comparison
// ─────────────────────────────────────────────────────────────────────────────
//
// Analytic (true) solution for semi-implicit Euler after N steps of dt:
//   v_y(N) = v0_y + g * N * dt
//   y(N)   = y0 + v_y(1)*dt + v_y(2)*dt + ... + v_y(N)*dt
//           = y0 + sum_{k=1}^{N} (v0_y + g*k*dt) * dt
//           = y0 + N*v0_y*dt + g*dt^2 * N*(N+1)/2
//
// This is NOT the same as the continuous solution:
//   y_continuous = y0 + v0*t + 0.5*g*t^2  (where t = N*dt)
//
// The difference is O(dt) per step, converging to the continuous solution
// as dt -> 0.  We test against the discrete recurrence, not the continuous
// solution, to avoid attributing discretisation error as a test failure.

TEST_CASE("Integration: velocity under gravity matches discrete recurrence", "[integration][projectile]")
{
    const float dt   = 1.f / 120.f;
    const float g    = -9.81f;
    const float v0y  = 0.f;
    const int   N    = 120;  // 1 second

    Engine e = makeEngine({0.f, g}, dt);
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {0.f, v0y});

    for (int i = 0; i < N; ++i) {
        e.clearForces();
        integrate(e, dt);
    }

    // Discrete velocity after N steps: v_y = v0_y + g * N * dt
    const float expected_vy = v0y + g * N * dt;
    REQUIRE(e.getVelocity(h).x == Approx(0.f).margin(1e-5f));
    REQUIRE(e.getVelocity(h).y == Approx(expected_vy).margin(1e-4f));
}

TEST_CASE("Integration: position under gravity matches discrete recurrence", "[integration][projectile]")
{
    const float dt  = 1.f / 120.f;
    const float g   = -9.81f;
    const float y0  = 100.f;
    const float v0y = 0.f;
    const int   N   = 120;

    Engine e = makeEngine({0.f, g}, dt);
    BodyHandle h = addDynamic(e, {0.f, y0}, {0.f, v0y});

    for (int i = 0; i < N; ++i) {
        e.clearForces();
        integrate(e, dt);
    }

    // Discrete y after N steps: y0 + g*dt^2 * N*(N+1)/2
    // (v0y = 0, so the v0*t term vanishes)
    const float expected_y = y0 + g * dt * dt * (N * (N + 1)) / 2.f;
    // Allow 0.1 m tolerance for float precision over 120 steps
    REQUIRE(e.getPosition(h).y == Approx(expected_y).margin(0.1f));
}

TEST_CASE("Integration: horizontal velocity is unchanged by vertical gravity", "[integration][projectile]")
{
    const float dt  = 1.f / 60.f;
    const float v0x = 5.f;

    Engine e = makeEngine({0.f, -9.81f});
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {v0x, 0.f});

    for (int i = 0; i < 60; ++i) {
        e.clearForces();
        integrate(e, dt);
    }

    // No horizontal gravity, no horizontal force -> vx is constant
    REQUIRE(e.getVelocity(h).x == Approx(v0x).margin(1e-4f));
    // x = v0x * N * dt = 5 * 60 * (1/60) = 5.0
    REQUIRE(e.getPosition(h).x == Approx(5.f).margin(1e-3f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Zero gravity — inertial motion
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Integration: zero gravity, initial velocity - body moves in straight line", "[integration][inertial]")
{
    Engine e = makeEngine({0.f, 0.f});
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {3.f, 4.f});

    const float dt = 1.f / 60.f;
    const int N = 60;
    for (int i = 0; i < N; ++i) {
        e.clearForces();
        integrate(e, dt);
    }

    // x = v0x * t = 3 * 1.0 = 3.0
    // y = v0y * t = 4 * 1.0 = 4.0
    REQUIRE(e.getPosition(h).x == Approx(3.f).margin(1e-3f));
    REQUIRE(e.getPosition(h).y == Approx(4.f).margin(1e-3f));
    // Velocity unchanged (no acceleration)
    REQUIRE(e.getVelocity(h).x == Approx(3.f).margin(1e-5f));
    REQUIRE(e.getVelocity(h).y == Approx(4.f).margin(1e-5f));
}

TEST_CASE("Integration: body at rest with no forces stays at rest", "[integration][inertial]")
{
    Engine e = makeEngine({0.f, 0.f});
    BodyHandle h = addDynamic(e, {1.f, 2.f}, {0.f, 0.f});

    for (int i = 0; i < 1000; ++i) {
        e.clearForces();
        integrate(e, 1.f / 120.f);
    }

    REQUIRE(e.getPosition(h).x == Approx(1.f));
    REQUIRE(e.getPosition(h).y == Approx(2.f));
    REQUIRE(e.getVelocity(h).x == 0.f);
    REQUIRE(e.getVelocity(h).y == 0.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// External force accumulation
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Integration: constant external force produces correct acceleration", "[integration][force]")
{
    // Apply a horizontal force F = 2 N to a 1 kg body.
    // Expected acceleration: a = F/m = 2 m/s^2.
    // After 1 step of dt: v_x = 2 * dt.
    // After N steps:      v_x = 2 * N * dt.

    Engine e = makeEngine({0.f, 0.f});  // no gravity
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {0.f, 0.f}, 1.f);

    const float dt  = 1.f / 120.f;
    const float F   = 2.f;
    const int   N   = 120;

    for (int i = 0; i < N; ++i) {
        e.clearForces();
        e.addForce(h, {F, 0.f});
        integrate(e, dt);
    }

    // v_x = a * N * dt = 2 * 120 * (1/120) = 2.0 m/s
    REQUIRE(e.getVelocity(h).x == Approx(2.f).margin(1e-4f));
    REQUIRE(e.getVelocity(h).y == Approx(0.f).margin(1e-5f));
}

TEST_CASE("Integration: force on 2kg body gives half acceleration", "[integration][force]")
{
    // F = 4 N, m = 2 kg -> a = 2 m/s^2.  Same expected v_x as above.
    Engine e = makeEngine({0.f, 0.f});
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {0.f, 0.f}, 2.f);

    const float dt = 1.f / 120.f;
    const int   N  = 120;

    for (int i = 0; i < N; ++i) {
        e.clearForces();
        e.addForce(h, {4.f, 0.f});
        integrate(e, dt);
    }

    REQUIRE(e.getVelocity(h).x == Approx(2.f).margin(1e-4f));
}

TEST_CASE("Integration: forces applied immediately before step are consumed and cleared", "[integration][force]")
{
    // Verifies that a force applied immediately before integrate() contributes to
    // that step's velocity and position, and is cleared afterwards so it does not
    // carry over to subsequent steps.

    Engine e = makeEngine({0.f, 0.f});
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {0.f, 0.f}, 1.f);

    const float dt = 1.f / 120.f;

    // Apply force immediately before integration
    e.addForce(h, {10.f, 0.f});
    integrate(e, dt);   // step 1: force = 10 N is consumed, then cleared

    // After step 1:
    // v_x = 0 + 10 * dt
    // x   = 0 + v_x * dt = 10 * dt^2
    const float expected_vx1 = 10.f * dt;
    const float expected_x1  = expected_vx1 * dt;
    REQUIRE(e.getVelocity(h).x == Approx(expected_vx1).margin(1e-5f));
    REQUIRE(e.getPosition(h).x == Approx(expected_x1).margin(1e-6f));

    // Force buffer must be cleared automatically after integration
    REQUIRE(e.forcesData()[0].x == 0.f);
    REQUIRE(e.forcesData()[0].y == 0.f);

    // Step 2 without adding new forces: velocity must not increase
    integrate(e, dt);
    REQUIRE(e.getVelocity(h).x == Approx(expected_vx1).margin(1e-5f));
    // Position advances by inertial motion: x2 = x1 + vx * dt
    REQUIRE(e.getPosition(h).x == Approx(expected_x1 + expected_vx1 * dt).margin(1e-6f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Semi-implicit Euler is NOT the same as explicit Euler
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Integration: position uses updated velocity (semi-implicit, not explicit)", "[integration][symplectic]")
{
    // For a body with v0=0 under constant acceleration a, after 1 step:
    //   Explicit Euler:       x = x0 + v0*dt      = x0
    //   Semi-implicit Euler:  x = x0 + (v0+a*dt)*dt = x0 + a*dt^2
    //
    // We verify the semi-implicit form: x > x0 after one step.

    Engine e = makeEngine({0.f, 0.f});  // no gravity
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {0.f, 0.f}, 1.f);

    const float dt = 1.f / 60.f;
    e.clearForces();
    e.addForce(h, {1.f, 0.f});  // 1 N -> 1 m/s^2 on 1 kg
    integrate(e, dt);

    // Semi-implicit: x = 0 + (0 + 1*dt)*dt = dt^2
    // Explicit:      x = 0 + 0*dt = 0
    const float expected_x = dt * dt;  // a * dt^2 = 1 * (1/60)^2
    REQUIRE(e.getPosition(h).x == Approx(expected_x).margin(1e-7f));
    // Also check velocity: v = 0 + 1*dt = dt
    REQUIRE(e.getVelocity(h).x == Approx(dt).margin(1e-7f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Multiple bodies
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Integration: multiple bodies integrate independently", "[integration][multi]")
{
    Engine e = makeEngine({0.f, -9.81f});

    // Body A: starts at y=100, v=0
    BodyHandle hA = addDynamic(e, {0.f, 100.f}, {0.f, 0.f});
    // Body B: starts at y=0, v=20 (thrown upward)
    BodyHandle hB = addDynamic(e, {0.f, 0.f}, {0.f, 20.f});

    const float dt = 1.f / 120.f;

    for (int i = 0; i < 120; ++i) {
        e.clearForces();
        integrate(e, dt);
    }

    // After 1 second:
    // A: fell from y=100 under gravity — must be below 100
    REQUIRE(e.getPosition(hA).y < 100.f);
    // B: thrown up at 20 m/s, after 1s vB = 20 - 9.81 = 10.19 m/s, still moving up
    REQUIRE(e.getVelocity(hB).y > 0.f);
    // B should be higher than start
    REQUIRE(e.getPosition(hB).y > 0.f);
}

// =============================================================================
// Task B additions
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
// Runtime gravity changes (Engine::setGravity)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: setGravity changes gravity for subsequent steps", "[engine][gravity]")
{
    // Start with downward gravity, run 1 step, change to zero, run another step.
    // The second step must show no further acceleration.

    Engine e = makeEngine({0.f, -9.81f});
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {0.f, 0.f});

    const float dt = 1.f / 60.f;

    e.clearForces();
    integrate(e, dt);  // step 1: gravity pulls down
    const float vy_after_step1 = e.getVelocity(h).y;
    REQUIRE(vy_after_step1 < 0.f);  // downward

    // Change gravity to zero
    e.setGravity({0.f, 0.f});
    REQUIRE(e.config().gravity.x == 0.f);
    REQUIRE(e.config().gravity.y == 0.f);

    e.clearForces();
    integrate(e, dt);  // step 2: no gravity — velocity must be unchanged
    REQUIRE(e.getVelocity(h).y == Approx(vy_after_step1));
}

TEST_CASE("Engine: setGravity reversed mid-simulation", "[engine][gravity]")
{
    // Flip gravity upward mid-flight.
    Engine e = makeEngine({0.f, -9.81f});
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {0.f, 0.f});
    const float dt = 1.f / 120.f;

    // 60 steps of downward gravity — body accelerates downward
    for (int i = 0; i < 60; ++i) {
        e.clearForces();
        integrate(e, dt);
    }
    const float vy_down = e.getVelocity(h).y;
    REQUIRE(vy_down < 0.f);

    // Flip gravity
    e.setGravity({0.f, +9.81f});

    // 60 more steps of upward gravity — body decelerates then accelerates upward
    for (int i = 0; i < 60; ++i) {
        e.clearForces();
        integrate(e, dt);
    }
    // After symmetric steps with equal magnitude but opposite gravity,
    // velocity should be approximately back near zero
    // (vy_down + 9.81*60*dt ≈ 0)
    REQUIRE(e.getVelocity(h).y == Approx(0.f).margin(0.05f));
}

TEST_CASE("Engine: setGravity rejects NaN", "[engine][gravity][validation]")
{
    Engine e = makeEngine({0.f, -9.81f});
    const Vec2 original = e.config().gravity;

    e.setGravity({std::numeric_limits<float>::quiet_NaN(), 0.f});

    // Gravity must be unchanged
    REQUIRE(e.config().gravity.x == Approx(original.x));
    REQUIRE(e.config().gravity.y == Approx(original.y));
}

TEST_CASE("Engine: setGravity rejects inf", "[engine][gravity][validation]")
{
    Engine e = makeEngine({0.f, -9.81f});
    const Vec2 original = e.config().gravity;

    e.setGravity({0.f, std::numeric_limits<float>::infinity()});

    REQUIRE(e.config().gravity.y == Approx(original.y));
}

// ─────────────────────────────────────────────────────────────────────────────
// Runtime velocity changes (Engine::setVelocity, with NaN validation)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: setVelocity takes effect on the next integrate step", "[engine][velocity][runtime]")
{
    Engine e = makeEngine({0.f, 0.f});  // no gravity
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {0.f, 0.f});

    // Give the body a velocity impulse mid-simulation
    e.setVelocity(h, {10.f, 0.f});

    const float dt = 1.f / 60.f;
    e.clearForces();
    integrate(e, dt);

    // x = 10 * dt after one step
    REQUIRE(e.getPosition(h).x == Approx(10.f * dt).margin(1e-5f));
    REQUIRE(e.getVelocity(h).x == Approx(10.f).margin(1e-5f));
}

TEST_CASE("Engine: setVelocity rejects NaN - velocity unchanged", "[engine][velocity][validation]")
{
    Engine e = makeEngine({0.f, 0.f});
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {3.f, 4.f});

    e.setVelocity(h, {std::numeric_limits<float>::quiet_NaN(), 1.f});

    // Velocity must be unchanged
    REQUIRE(e.getVelocity(h).x == Approx(3.f));
    REQUIRE(e.getVelocity(h).y == Approx(4.f));
}

TEST_CASE("Engine: setVelocity rejects inf - velocity unchanged", "[engine][velocity][validation]")
{
    Engine e = makeEngine({0.f, 0.f});
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {1.f, 2.f});

    e.setVelocity(h, {1.f, std::numeric_limits<float>::infinity()});

    REQUIRE(e.getVelocity(h).x == Approx(1.f));
    REQUIRE(e.getVelocity(h).y == Approx(2.f));
}

TEST_CASE("Engine: setPosition rejects NaN - position unchanged", "[engine][position][validation]")
{
    Engine e = makeEngine({0.f, 0.f});
    BodyHandle h = addDynamic(e, {5.f, 7.f});

    e.setPosition(h, {std::numeric_limits<float>::quiet_NaN(), 0.f});

    REQUIRE(e.getPosition(h).x == Approx(5.f));
    REQUIRE(e.getPosition(h).y == Approx(7.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Deterministic headless scene test
//
// Scene: two dynamic bodies and one static floor marker.
// All inputs are fixed. Resulting positions and velocities after a known
// number of steps are verified against precomputed expected values.
//
// Expected values derived from the semi-implicit Euler discrete recurrence:
//   v_y(N) = v0_y + g * N * dt
//   y(N)   = y0 + g * dt^2 * N*(N+1)/2  (for v0_y = 0)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Integration: deterministic headless scene - fixed 120 steps", "[integration][deterministic]")
{
    // Scene parameters (fixed, deterministic)
    constexpr float g  = -9.81f;
    constexpr float dt = 1.f / 120.f;
    constexpr int   N  = 120;   // exactly 1 simulated second

    WorldConfig cfg;
    cfg.gravity   = {0.f, g};
    cfg.fixedDt   = dt;
    cfg.maxBodies = 8;
    Engine e{cfg};

    // Body A: dropped from height 50, no initial velocity, mass 1 kg
    BodyDesc dA;
    dA.position = {0.f, 50.f};
    dA.velocity = {0.f, 0.f};
    dA.mass     = 1.f;
    BodyHandle hA = e.addBody(dA);

    // Body B: launched horizontally at 5 m/s from origin, mass 2 kg
    BodyDesc dB;
    dB.position = {0.f, 0.f};
    dB.velocity = {5.f, 0.f};
    dB.mass     = 2.f;
    BodyHandle hB = e.addBody(dB);

    // Static floor marker at y=-10 (must not move)
    BodyDesc dF;
    dF.position = {0.f, -10.f};
    dF.mass     = 0.f;   // static
    dF.radius   = 0.5f;
    BodyHandle hF = e.addBody(dF);

    // Run exactly N steps
    for (int i = 0; i < N; ++i) {
        e.clearForces();
        integrate(e, dt);
    }

    // ── Body A: free fall from y=50 ──────────────────────────────────────────
    // v_y(N)  = 0 + g * N * dt = -9.81 * 120 * (1/120) = -9.81 m/s
    const float A_vy_expected = g * N * dt;
    // y(N)    = 50 + g * dt^2 * N*(N+1)/2
    const float A_y_expected  = 50.f + g * dt * dt * (N * (N + 1)) / 2.f;
    // x is unchanged (no horizontal force or gravity)
    REQUIRE(e.getVelocity(hA).x == Approx(0.f).margin(1e-4f));
    REQUIRE(e.getVelocity(hA).y == Approx(A_vy_expected).margin(1e-3f));
    REQUIRE(e.getPosition(hA).x == Approx(0.f).margin(1e-4f));
    REQUIRE(e.getPosition(hA).y == Approx(A_y_expected).margin(0.1f));

    // ── Body B: horizontal launch, gravity pulls it down ─────────────────────
    // vx is constant (no horizontal force): vx = 5 m/s
    // x(N) = 0 + vx * N * dt = 5 * 1.0 = 5.0 m
    // vy(N) = g * N * dt = -9.81 m/s  (same as A, same gravity, v0y=0)
    // y(N)  = g * dt^2 * N*(N+1)/2    (same as A with y0=0)
    const float B_vy_expected = g * N * dt;
    const float B_y_expected  = g * dt * dt * (N * (N + 1)) / 2.f;
    REQUIRE(e.getVelocity(hB).x == Approx(5.f).margin(1e-3f));
    REQUIRE(e.getVelocity(hB).y == Approx(B_vy_expected).margin(1e-3f));
    REQUIRE(e.getPosition(hB).x == Approx(5.f).margin(1e-2f));
    REQUIRE(e.getPosition(hB).y == Approx(B_y_expected).margin(0.1f));

    // ── Static floor marker: must not have moved ──────────────────────────────
    REQUIRE(e.getPosition(hF).x == Approx(0.f));
    REQUIRE(e.getPosition(hF).y == Approx(-10.f));
    REQUIRE(e.getVelocity(hF).x == 0.f);
    REQUIRE(e.getVelocity(hF).y == 0.f);

    // ── All handles still valid ───────────────────────────────────────────────
    REQUIRE(e.isValidHandle(hA));
    REQUIRE(e.isValidHandle(hB));
    REQUIRE(e.isValidHandle(hF));
    REQUIRE(e.bodyCount() == 3);
}

TEST_CASE("Integration: deterministic scene is reproducible across two runs", "[integration][deterministic]")
{
    // Run the same scene twice with identical parameters.
    // Both runs must produce bit-for-bit identical results.
    // This verifies that there is no hidden state, random seed, or
    // order-dependent behaviour affecting the simulation.

    auto runScene = [](float dt, int N) -> std::pair<Vec2, Vec2> {
        WorldConfig cfg;
        cfg.gravity   = {0.f, -9.81f};
        cfg.fixedDt   = dt;
        cfg.maxBodies = 4;
        Engine e{cfg};

        BodyDesc d;
        d.position = {2.f, 20.f};
        d.velocity = {1.f, 5.f};
        d.mass     = 3.f;
        BodyHandle h = e.addBody(d);

        for (int i = 0; i < N; ++i) {
            e.clearForces();
            integrate(e, dt);
        }
        return {e.getPosition(h), e.getVelocity(h)};
    };

    const float dt = 1.f / 120.f;
    const int   N  = 200;

    auto [pos1, vel1] = runScene(dt, N);
    auto [pos2, vel2] = runScene(dt, N);

    // Exact float equality — same ops, same order, same result
    REQUIRE(pos1.x == pos2.x);
    REQUIRE(pos1.y == pos2.y);
    REQUIRE(vel1.x == vel2.x);
    REQUIRE(vel1.y == vel2.y);
}

// ─────────────────────────────────────────────────────────────────────────────
// Force accumulation semantics clarification tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Integration: force accumulates across multiple addForce calls in same step", "[integration][force][semantics]")
{
    // Two addForce calls in the same step must be additive.
    Engine e = makeEngine({0.f, 0.f});
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {0.f, 0.f}, 1.f);

    const float dt = 1.f / 60.f;
    e.clearForces();
    e.addForce(h, {3.f, 0.f});
    e.addForce(h, {2.f, 0.f});  // total force = 5 N
    integrate(e, dt);

    // v_x = 5 * dt (force / mass = 5, times dt)
    REQUIRE(e.getVelocity(h).x == Approx(5.f * dt).margin(1e-6f));
}

TEST_CASE("Integration: clearForces before each step prevents carry-over", "[integration][force][semantics]")
{
    // Correct step loop: clearForces → addForce → integrate.
    // The second step must only see the force applied in that step, not the first.
    Engine e = makeEngine({0.f, 0.f});
    BodyHandle h = addDynamic(e, {0.f, 0.f}, {0.f, 0.f}, 1.f);

    const float dt = 1.f / 120.f;
    const float F  = 6.f;

    // Step 1
    e.clearForces();
    e.addForce(h, {F, 0.f});
    integrate(e, dt);
    const float vx_after_1 = e.getVelocity(h).x;
    REQUIRE(vx_after_1 == Approx(F * dt).margin(1e-6f));

    // Step 2 — clearForces first, then NO addForce
    e.clearForces();
    integrate(e, dt);
    // Velocity must increase by zero (no force this step, no gravity)
    REQUIRE(e.getVelocity(h).x == Approx(vx_after_1).margin(1e-6f));
}

TEST_CASE("Integration: forces do not affect static bodies even without clearForces", "[integration][force][semantics]")
{
    // Verify the invariant: static bodies are immune to forces regardless of
    // what is in the force buffer.
    Engine e = makeEngine({0.f, 0.f});
    BodyHandle hs = addStatic(e, {3.f, 3.f});

    // Manually write a non-zero force via forcesMut (bypasses addForce guard)
    // to test that integration itself skips static bodies — not just addForce.
    Vec2* forces = e.forcesMut();
    forces[0] = {999.f, 999.f};   // SoA slot 0 (the only body)

    integrate(e, 1.f / 60.f);

    REQUIRE(e.getPosition(hs) == Vec2{3.f, 3.f});
    REQUIRE(e.getVelocity(hs) == Vec2{});
}
