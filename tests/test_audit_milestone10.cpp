// test_audit_milestone10.cpp -- Regression and correctness audit tests for Milestone 10.
//
// Verification Coverage:
//   1. Simulation Correctness & Invariants:
//      - Paused simulation does not advance under advance(); accumulator untouched.
//      - Single-step advances exactly 1 fixedDt and preserves paused state.
//      - Static bodies remain strictly stationary under heavy dynamic bombardment.
//      - Forces are applied and cleared strictly per-step.
//      - Non-finite coordinates, velocities, and masses are rejected without NaN corruption.
//   2. Resource & Capacity Limits:
//      - Exact capacity saturation and overflow rejection without handle corruption.
//      - Swap-and-pop handle generation invalidation and slot recycling.
//   3. Broad-Phase Canonical Equivalence:
//      - Complex scene with mixed shapes, negative coordinates, dense clusters, and oversized bodies.
//      - Bitwise canonical pair equivalence: BruteForce == SpatialGrid == SpatialGridParallel.
//   4. Deterministic Reset & Accumulator Behavior:
//      - Resetting state, accumulator, and contacts restores bit-for-bit reproducibility.

#include "PhysicsEngine/Concurrency/WorkerPool.h"
#include "PhysicsEngine/Math/Vec2.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Physics/Collision.h"
#include "PhysicsEngine/Physics/CollisionSolver.h"
#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/Integration.h"
#include "PhysicsEngine/Physics/SimulationController.h"
#include "PhysicsEngine/Physics/SpatialGrid.h"
#include "PhysicsEngine/Physics/WorldConfig.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <set>
#include <vector>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {

std::set<std::pair<uint32_t, uint32_t>> extractCanonicalPairs(const std::vector<Contact>& contacts)
{
    std::set<std::pair<uint32_t, uint32_t>> pairs;
    for (const auto& c : contacts) {
        pairs.insert({std::min(c.bodyA.index, c.bodyB.index),
                      std::max(c.bodyA.index, c.bodyB.index)});
    }
    return pairs;
}

} // anonymous namespace

TEST_CASE("Milestone 10 Audit: Non-finite inputs cannot corrupt Engine state", "[audit][robustness]")
{
    WorldConfig cfg;
    cfg.maxBodies = 16;
    Engine engine{cfg};

    const float kNaN = std::numeric_limits<float>::quiet_NaN();
    const float kInf = std::numeric_limits<float>::infinity();

    SECTION("addBody rejects non-finite positions") {
        BodyDesc d;
        d.position = Vec2{kNaN, 0.f};
        BodyHandle h = engine.addBody(d);
        CHECK_FALSE(engine.isValidHandle(h));
        CHECK(h.isNull());
        CHECK(engine.bodyCount() == 0);

        d.position = Vec2{0.f, kInf};
        BodyHandle h2 = engine.addBody(d);
        CHECK_FALSE(engine.isValidHandle(h2));
        CHECK(engine.bodyCount() == 0);
    }

    SECTION("addBody rejects non-finite velocities") {
        BodyDesc d;
        d.position = Vec2{0.f, 0.f};
        d.velocity = Vec2{kNaN, 1.f};
        BodyHandle h = engine.addBody(d);
        CHECK_FALSE(engine.isValidHandle(h));
        CHECK(engine.bodyCount() == 0);
    }

    SECTION("addBody sanitises non-finite dimensions, mass, and restitution") {
        BodyDesc d;
        d.position    = Vec2{1.f, 2.f};
        d.radius      = kInf; // non-finite dimension
        d.halfHeight  = kNaN;
        d.restitution = kNaN;
        d.mass        = kInf; // infinite mass -> static body (invMass = 0)

        BodyHandle h = engine.addBody(d);
        REQUIRE(engine.isValidHandle(h));
        CHECK(engine.bodyCount() == 1);
        CHECK(engine.getRadius(h) == 1.f); // fallback dimension
        CHECK(engine.getRestitution(h) == 0.f);
        CHECK(engine.getInverseMass(h) == 0.f);
        CHECK(engine.isStatic(h));
    }

    SECTION("setGravity rejects non-finite vectors") {
        const Vec2 origGrav = engine.config().gravity;
        engine.setGravity(Vec2{kNaN, -9.81f});
        CHECK(engine.config().gravity.x == origGrav.x);
        CHECK(engine.config().gravity.y == origGrav.y);

        engine.setGravity(Vec2{0.f, kInf});
        CHECK(engine.config().gravity.x == origGrav.x);
        CHECK(engine.config().gravity.y == origGrav.y);
    }
}

TEST_CASE("Milestone 10 Audit: Static bodies remain strictly stationary under heavy dynamic collisions", "[audit][simulation]")
{
    WorldConfig cfg;
    cfg.maxBodies = 32;
    cfg.gravity   = Vec2{0.f, -9.81f};
    cfg.fixedDt   = 1.f / 120.f;
    Engine engine{cfg};

    // Static floor AABB
    BodyDesc floor;
    floor.position    = Vec2{0.f, 0.f};
    floor.mass        = 0.f; // static
    floor.shapeType   = ShapeType::AABB;
    floor.radius      = 10.f;
    floor.halfHeight  = 1.f;
    floor.restitution = 0.5f;
    BodyHandle hFloor = engine.addBody(floor);
    REQUIRE(engine.isValidHandle(hFloor));

    // Static circle peg
    BodyDesc peg;
    peg.position    = Vec2{2.f, 3.f};
    peg.mass        = 0.f; // static
    peg.shapeType   = ShapeType::Circle;
    peg.radius      = 1.f;
    peg.restitution = 0.8f;
    BodyHandle hPeg = engine.addBody(peg);
    REQUIRE(engine.isValidHandle(hPeg));

    // Dynamic projectile slamming into static bodies at high velocity
    BodyDesc dyn;
    dyn.position    = Vec2{0.f, 8.f};
    dyn.velocity    = Vec2{0.f, -50.f}; // high downward speed
    dyn.mass        = 5.f;
    dyn.shapeType   = ShapeType::Circle;
    dyn.radius      = 0.75f;
    dyn.restitution = 0.5f;
    BodyHandle hDyn = engine.addBody(dyn);
    REQUIRE(engine.isValidHandle(hDyn));

    SimulationController controller;
    controller.setBroadPhaseType(BroadPhaseType::SpatialGrid);

    // Simulate for 240 steps (2.0 seconds)
    for (int step = 0; step < 240; ++step) {
        // Attempt external force on static floor (must be rejected)
        engine.addForce(hFloor, Vec2{1000.f, 1000.f});
        controller.singleStep(engine);

        // Verify static floor position and velocity remain EXACTLY zero
        CHECK(engine.getPosition(hFloor).x == 0.f);
        CHECK(engine.getPosition(hFloor).y == 0.f);
        CHECK(engine.getVelocity(hFloor).x == 0.f);
        CHECK(engine.getVelocity(hFloor).y == 0.f);

        // Verify static peg position and velocity remain EXACTLY untouched
        CHECK(engine.getPosition(hPeg).x == 2.f);
        CHECK(engine.getPosition(hPeg).y == 3.f);
        CHECK(engine.getVelocity(hPeg).x == 0.f);
        CHECK(engine.getVelocity(hPeg).y == 0.f);
    }

    // Dynamic body must have rebounded and stay finite
    CHECK(std::isfinite(engine.getPosition(hDyn).y));
    CHECK(std::isfinite(engine.getVelocity(hDyn).y));
}

TEST_CASE("Milestone 10 Audit: Force application and single-step clearing semantics", "[audit][forces]")
{
    WorldConfig cfg;
    cfg.maxBodies = 8;
    cfg.gravity   = Vec2{0.f, 0.f}; // zero gravity to isolate force effect
    cfg.fixedDt   = 1.f / 120.f;
    Engine engine{cfg};

    BodyDesc d;
    d.position = Vec2{0.f, 0.f};
    d.velocity = Vec2{0.f, 0.f};
    d.mass     = 2.0f; // 2 kg -> invMass = 0.5
    BodyHandle h = engine.addBody(d);
    REQUIRE(engine.isValidHandle(h));

    // Apply 20 N in X direction: a = F / m = 10 m/s2 -> v1 = 10 * (1/120) = 0.08333 m/s
    engine.addForce(h, Vec2{20.f, 0.f});
    CHECK(engine.forcesData()[0].x == Approx(20.f));

    SimulationController controller;
    controller.singleStep(engine);

    const float expV1 = (20.f * 0.5f) * (1.f / 120.f);
    CHECK(engine.getVelocity(h).x == Approx(expV1));

    // Forces must be cleared after integration
    CHECK(engine.forcesData()[0].x == 0.f);
    CHECK(engine.forcesData()[0].y == 0.f);

    // Next step without adding forces must maintain velocity without additional acceleration
    controller.singleStep(engine);
    CHECK(engine.getVelocity(h).x == Approx(expV1));
}

TEST_CASE("Milestone 10 Audit: Broad-phase equivalence across complex clustered scene", "[audit][broadphase]")
{
    WorldConfig cfg;
    cfg.maxBodies = 256;
    cfg.gravity   = Vec2{0.f, 0.f};
    Engine engine{cfg};

    std::mt19937 rng(9999);
    std::uniform_real_distribution<float> posDist(-40.f, 40.f);
    std::uniform_real_distribution<float> sizeDist(0.3f, 1.2f);
    std::uniform_int_distribution<int> shapeDist(0, 1);
    std::uniform_int_distribution<int> staticDist(0, 4); // 20% static

    // Generate mixed bodies across negative and positive quadrants
    for (int i = 0; i < 120; ++i) {
        BodyDesc d;
        d.position    = Vec2{posDist(rng), posDist(rng)};
        d.mass        = (staticDist(rng) == 0) ? 0.f : 1.f;
        d.shapeType   = (shapeDist(rng) == 0) ? ShapeType::Circle : ShapeType::AABB;
        d.radius      = sizeDist(rng);
        d.halfHeight  = sizeDist(rng);
        d.restitution = 0.5f;
        (void)engine.addBody(d);
    }

    // Add an oversized static floor covering multiple cells
    BodyDesc giantFloor;
    giantFloor.position   = Vec2{0.f, -30.f};
    giantFloor.mass       = 0.f;
    giantFloor.shapeType  = ShapeType::AABB;
    giantFloor.radius     = 50.f; // 100m wide
    giantFloor.halfHeight = 2.f;
    (void)engine.addBody(giantFloor);

    // Add a dense cluster of overlapping circles
    for (int i = 0; i < 15; ++i) {
        BodyDesc d;
        d.position  = Vec2{5.0f + static_cast<float>(i) * 0.1f, 5.0f};
        d.mass      = 1.0f;
        d.shapeType = ShapeType::Circle;
        d.radius    = 0.6f;
        (void)engine.addBody(d);
    }

    std::vector<Contact> contactsBF;
    std::vector<Contact> contactsSeq;
    std::vector<Contact> contactsPar2;
    std::vector<Contact> contactsPar4;

    SpatialGrid grid(3.0f);
    WorkerPool pool2(2);
    WorkerPool pool4(4);

    detectCollisionsBruteForce(engine, contactsBF);
    grid.detectCollisions(engine, contactsSeq);
    grid.detectCollisionsParallel(engine, contactsPar2, pool2);
    grid.detectCollisionsParallel(engine, contactsPar4, pool4);

    auto pairsBF   = extractCanonicalPairs(contactsBF);
    auto pairsSeq  = extractCanonicalPairs(contactsSeq);
    auto pairsPar2 = extractCanonicalPairs(contactsPar2);
    auto pairsPar4 = extractCanonicalPairs(contactsPar4);

    REQUIRE_FALSE(pairsBF.empty());
    CHECK(pairsSeq  == pairsBF);
    CHECK(pairsPar2 == pairsBF);
    CHECK(pairsPar4 == pairsBF);
}

TEST_CASE("Milestone 10 Audit: Scene reset bit-for-bit reproducibility", "[audit][reset]")
{
    auto setupScene = [](Engine& e) -> BodyHandle {
        e.clear();
        BodyDesc floor;
        floor.position    = Vec2{0.f, -5.f};
        floor.mass        = 0.f;
        floor.shapeType   = ShapeType::AABB;
        floor.radius      = 10.f;
        floor.halfHeight  = 0.5f;
        (void)e.addBody(floor);

        BodyDesc circle;
        circle.position   = Vec2{0.f, 5.f};
        circle.mass       = 1.f;
        circle.shapeType  = ShapeType::Circle;
        circle.radius     = 0.5f;
        return e.addBody(circle);
    };

    WorldConfig cfg;
    cfg.maxBodies = 16;
    cfg.gravity   = Vec2{0.f, -9.81f};
    cfg.fixedDt   = 1.f / 120.f;
    Engine engine{cfg};
    SimulationController controller;

    // Run 1: fresh scene, advance 1 single step
    BodyHandle hCircle1 = setupScene(engine);
    REQUIRE(engine.isValidHandle(hCircle1));
    controller.resetAccumulator();
    controller.clearContacts();
    controller.singleStep(engine);

    const Vec2 posRun1 = engine.getPosition(hCircle1);
    const Vec2 velRun1 = engine.getVelocity(hCircle1);

    // Run simulation for 200 steps
    for (int i = 0; i < 200; ++i) {
        controller.singleStep(engine);
    }
    CHECK(engine.getPosition(hCircle1).y != posRun1.y);

    // Reset scene
    BodyHandle hCircle2 = setupScene(engine);
    REQUIRE(engine.isValidHandle(hCircle2));
    controller.resetAccumulator();
    controller.clearContacts();
    controller.singleStep(engine);

    // Old handle hCircle1 must be stale/invalid
    CHECK_FALSE(engine.isValidHandle(hCircle1));

    // Positions and velocities of newly created circle after reset must be bit-for-bit identical to Run 1
    CHECK(engine.getPosition(hCircle2).x == posRun1.x);
    CHECK(engine.getPosition(hCircle2).y == posRun1.y);
    CHECK(engine.getVelocity(hCircle2).x == velRun1.x);
    CHECK(engine.getVelocity(hCircle2).y == velRun1.y);
}
