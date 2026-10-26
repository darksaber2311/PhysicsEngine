// tests/test_parallel_broadphase.cpp -- Unit & regression tests for parallel broad-phase.

#include "PhysicsEngine/Concurrency/WorkerPool.h"
#include "PhysicsEngine/Math/Vec2.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Physics/Collision.h"
#include "PhysicsEngine/Physics/Contact.h"
#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/SimulationController.h"
#include "PhysicsEngine/Physics/SpatialGrid.h"
#include "PhysicsEngine/Physics/WorldConfig.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <random>
#include <set>
#include <utility>
#include <vector>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {

Engine makeEngine(uint32_t maxBodies = 1024)
{
    WorldConfig cfg;
    cfg.maxBodies = maxBodies;
    cfg.gravity   = {0.f, 0.f};
    return Engine{cfg};
}

BodyDesc makeCircle(Vec2 pos, float radius, float mass = 1.f)
{
    BodyDesc d;
    d.position    = pos;
    d.mass        = mass;
    d.shapeType   = ShapeType::Circle;
    d.radius      = radius;
    d.halfHeight  = 0.f;
    d.restitution = 0.5f;
    return d;
}

BodyDesc makeAABB(Vec2 pos, float halfW, float halfH, float mass = 1.f)
{
    BodyDesc d;
    d.position    = pos;
    d.mass        = mass;
    d.shapeType   = ShapeType::AABB;
    d.radius      = halfW;
    d.halfHeight  = halfH;
    d.restitution = 0.5f;
    return d;
}

using CanonicalPair = std::pair<uint32_t, uint32_t>;

std::set<CanonicalPair> extractCanonicalPairs(const std::vector<Contact>& contacts)
{
    std::set<CanonicalPair> pairs;
    for (const auto& c : contacts) {
        pairs.insert({std::min(c.bodyA.index, c.bodyB.index),
                      std::max(c.bodyA.index, c.bodyB.index)});
    }
    return pairs;
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// 1. Sequential vs Parallel Equivalence on Random Scenes
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Parallel Broad-Phase: matches sequential SpatialGrid and BruteForce on random scenes", "[broadphase][parallel]")
{
    Engine engine = makeEngine(512);
    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> posDist(-50.f, 50.f);
    std::uniform_real_distribution<float> sizeDist(0.5f, 2.0f);
    std::uniform_int_distribution<int> shapeDist(0, 1);
    std::uniform_int_distribution<int> staticDist(0, 9); // 10% static

    // Generate 300 bodies (exceeding default sequentialThreshold 256)
    for (int i = 0; i < 300; ++i) {
        Vec2 pos{posDist(rng), posDist(rng)};
        float mass = (staticDist(rng) == 0) ? 0.f : 1.f;
        if (shapeDist(rng) == 0) {
            (void)engine.addBody(makeCircle(pos, sizeDist(rng), mass));
        } else {
            (void)engine.addBody(makeAABB(pos, sizeDist(rng), sizeDist(rng), mass));
        }
    }

    // 1. Brute force reference
    std::vector<Contact> contactsBF;
    detectCollisionsBruteForce(engine, contactsBF);
    auto pairsBF = extractCanonicalPairs(contactsBF);

    // 2. Sequential Spatial Grid
    SpatialGrid gridSeq(3.0f);
    std::vector<Contact> contactsSeq;
    gridSeq.detectCollisions(engine, contactsSeq);
    auto pairsSeq = extractCanonicalPairs(contactsSeq);

    // 3. Parallel Spatial Grid
    WorkerPool pool(4);
    SpatialGrid gridPar(3.0f);
    gridPar.setSequentialThreshold(100); // Allow parallel path
    std::vector<Contact> contactsPar;
    gridPar.detectCollisionsParallel(engine, contactsPar, pool);
    auto pairsPar = extractCanonicalPairs(contactsPar);

    REQUIRE(pairsSeq == pairsBF);
    REQUIRE(pairsPar == pairsBF);
    REQUIRE(contactsPar.size() == contactsSeq.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Multiple Worker Counts Produce Identical Pair Sets
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Parallel Broad-Phase: multiple worker counts produce identical pairs", "[broadphase][parallel]")
{
    Engine engine = makeEngine(512);
    std::mt19937 rng(2026);
    std::uniform_real_distribution<float> posDist(-40.f, 40.f);
    std::uniform_real_distribution<float> sizeDist(0.5f, 1.5f);
    std::uniform_int_distribution<int> shapeDist(0, 1);

    for (int i = 0; i < 200; ++i) {
        Vec2 pos{posDist(rng), posDist(rng)};
        if (shapeDist(rng) == 0) {
            (void)engine.addBody(makeCircle(pos, sizeDist(rng)));
        } else {
            (void)engine.addBody(makeAABB(pos, sizeDist(rng), sizeDist(rng)));
        }
    }

    SpatialGrid grid(2.5f);
    grid.setSequentialThreshold(50);

    std::vector<Contact> contactsSeq;
    grid.detectCollisions(engine, contactsSeq);
    auto baselinePairs = extractCanonicalPairs(contactsSeq);

    const uint32_t threadCounts[] = { 1, 2, 4, 8 };
    for (uint32_t tCount : threadCounts) {
        WorkerPool pool(tCount);
        std::vector<Contact> contactsPar;
        grid.detectCollisionsParallel(engine, contactsPar, pool);
        auto testPairs = extractCanonicalPairs(contactsPar);

        REQUIRE(testPairs == baselinePairs);
        REQUIRE(contactsPar.size() == contactsSeq.size());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Determinism Across Repeated Runs
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Parallel Broad-Phase: repeated runs on same scene are 100% deterministic", "[broadphase][parallel]")
{
    Engine engine = makeEngine(512);
    std::mt19937 rng(999);
    std::uniform_real_distribution<float> posDist(-30.f, 30.f);
    std::uniform_real_distribution<float> sizeDist(0.4f, 1.2f);

    for (int i = 0; i < 150; ++i) {
        (void)engine.addBody(makeCircle({posDist(rng), posDist(rng)}, sizeDist(rng)));
    }

    WorkerPool pool(4);
    SpatialGrid grid(2.0f);
    grid.setSequentialThreshold(50);

    std::vector<Contact> firstRun;
    grid.detectCollisionsParallel(engine, firstRun, pool);
    REQUIRE_FALSE(firstRun.empty());

    // Run 20 times in a row: contacts must be identical in size, handles, normal, and penetration
    for (int iter = 0; iter < 20; ++iter) {
        std::vector<Contact> runContacts;
        grid.detectCollisionsParallel(engine, runContacts, pool);

        REQUIRE(runContacts.size() == firstRun.size());
        for (size_t k = 0; k < firstRun.size(); ++k) {
            REQUIRE(runContacts[k].bodyA == firstRun[k].bodyA);
            REQUIRE(runContacts[k].bodyB == firstRun[k].bodyB);
            REQUIRE(runContacts[k].normal.x == Approx(firstRun[k].normal.x).margin(1e-5f));
            REQUIRE(runContacts[k].normal.y == Approx(firstRun[k].normal.y).margin(1e-5f));
            REQUIRE(runContacts[k].penetration == Approx(firstRun[k].penetration).margin(1e-5f));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Dense Clustered Scenes
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Parallel Broad-Phase: dense clustered scenes resolve without data races", "[broadphase][parallel]")
{
    Engine engine = makeEngine(256);
    // 50 bodies tightly clustered within a 2x2 meter area
    std::mt19937 rng(777);
    std::uniform_real_distribution<float> posDist(-1.0f, 1.0f);

    for (int i = 0; i < 50; ++i) {
        (void)engine.addBody(makeCircle({posDist(rng), posDist(rng)}, 0.4f));
    }

    WorkerPool pool(4);
    SpatialGrid grid(1.0f);
    grid.setSequentialThreshold(10); // Force parallel

    std::vector<Contact> contactsSeq;
    grid.detectCollisions(engine, contactsSeq);

    std::vector<Contact> contactsPar;
    grid.detectCollisionsParallel(engine, contactsPar, pool);

    REQUIRE(extractCanonicalPairs(contactsPar) == extractCanonicalPairs(contactsSeq));
    REQUIRE(contactsPar.size() == contactsSeq.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Oversized Bodies Handling in Parallel
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Parallel Broad-Phase: oversized bodies are detected correctly in parallel", "[broadphase][parallel][oversized]")
{
    Engine engine = makeEngine(128);
    // Huge 2000m floor
    BodyHandle hFloor = engine.addBody(makeAABB({0.f, -2.f}, 1000.f, 2.f, 0.f));

    // Scatter circles along the floor
    std::vector<BodyHandle> circles;
    for (float x : {-900.f, -400.f, 0.f, 400.f, 900.f}) {
        circles.push_back(engine.addBody(makeCircle({x, 0.5f}, 0.8f)));
    }
    // High circle not colliding
    (void)engine.addBody(makeCircle({0.f, 10.f}, 0.8f));

    WorkerPool pool(4);
    SpatialGrid grid(2.0f);
    grid.setSequentialThreshold(1); // Force parallel even on small body count

    std::vector<Contact> contactsPar;
    grid.detectCollisionsParallel(engine, contactsPar, pool);

    std::vector<Contact> contactsBF;
    detectCollisionsBruteForce(engine, contactsBF);

    auto pairsPar = extractCanonicalPairs(contactsPar);
    auto pairsBF  = extractCanonicalPairs(contactsBF);

    REQUIRE(pairsPar == pairsBF);
    REQUIRE(pairsPar.size() == 5);
    for (const auto& c : contactsPar) {
        REQUIRE((c.bodyA == hFloor || c.bodyB == hFloor));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Empty, Single-Body, and Below-Threshold Scenes
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Parallel Broad-Phase: empty, single-body, and below-threshold scenes", "[broadphase][parallel]")
{
    WorkerPool pool(4);
    SpatialGrid grid(2.0f);
    std::vector<Contact> contacts;

    // 0 bodies
    {
        Engine engine = makeEngine(16);
        grid.detectCollisionsParallel(engine, contacts, pool);
        REQUIRE(contacts.empty());
        REQUIRE(grid.lastCandidatePairCount() == 0);
    }

    // 1 body
    {
        Engine engine = makeEngine(16);
        (void)engine.addBody(makeCircle({0.f, 0.f}, 1.f));
        grid.detectCollisionsParallel(engine, contacts, pool);
        REQUIRE(contacts.empty());
        REQUIRE(grid.lastCandidatePairCount() == 0);
    }

    // Below sequential threshold
    {
        Engine engine = makeEngine(32);
        (void)engine.addBody(makeCircle({0.f, 0.f}, 1.f));
        (void)engine.addBody(makeCircle({1.2f, 0.f}, 1.f));
        grid.setSequentialThreshold(256); // 2 bodies < 256 -> runs sequential fallback
        grid.detectCollisionsParallel(engine, contacts, pool);
        REQUIRE(contacts.size() == 1);
        REQUIRE(grid.lastCandidatePairCount() == 1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. Non-Finite Values, Coordinate Overflow, and Removed Bodies
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Parallel Broad-Phase: non-finite values and stale handles are safely handled", "[broadphase][parallel][safety]")
{
    const float nanF = std::numeric_limits<float>::quiet_NaN();
    const float infF = std::numeric_limits<float>::infinity();

    Engine engine = makeEngine(32);
    BodyHandle h1 = engine.addBody(makeCircle({0.f, 0.f}, 1.f));
    BodyHandle h2 = engine.addBody(makeCircle({1.2f, 0.f}, 1.f));
    (void)engine.addBody(makeCircle({nanF, 0.f}, 1.f));
    (void)engine.addBody(makeCircle({0.f, infF}, 1.f));
    (void)engine.addBody(makeCircle({2e8f, 0.f}, 1.f)); // Beyond kMaxCoord

    WorkerPool pool(4);
    SpatialGrid grid(2.0f);
    grid.setSequentialThreshold(1);

    auto contacts = grid.detectCollisionsParallel(engine, pool);
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].bodyA == h1);
    REQUIRE(contacts[0].bodyB == h2);

    // Remove h2 and step again
    engine.removeBody(h2);
    contacts = grid.detectCollisionsParallel(engine, pool);
    REQUIRE(contacts.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. SimulationController Integration with BroadPhaseType::SpatialGridParallel
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Parallel Broad-Phase: SimulationController executes parallel broad-phase smoothly", "[controller][parallel]")
{
    Engine engine = makeEngine(64);
    // Two dynamic circles moving towards each other along X
    BodyHandle hA = engine.addBody(makeCircle({-1.f, 0.f}, 1.f, 1.f));
    BodyHandle hB = engine.addBody(makeCircle({ 1.f, 0.f}, 1.f, 1.f));
    engine.setVelocity(hA, {2.f, 0.f});
    engine.setVelocity(hB, {-2.f, 0.f});

    WorkerPool pool(4);
    SimulationController controller;
    controller.setWorkerPool(&pool);
    controller.setBroadPhaseType(BroadPhaseType::SpatialGridParallel);
    controller.spatialGrid().setSequentialThreshold(1); // Force parallel path

    REQUIRE(controller.broadPhaseType() == BroadPhaseType::SpatialGridParallel);
    REQUIRE(controller.workerPool() == &pool);

    // Step simulation
    controller.step(engine);

    // Bodies collide and bounce apart
    REQUIRE(controller.lastContacts().size() == 1);
    REQUIRE(engine.getVelocity(hA).x < 0.f);
    REQUIRE(engine.getVelocity(hB).x > 0.f);
}
