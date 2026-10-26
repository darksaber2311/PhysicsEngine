// test_spatial_grid.cpp -- Catch2 tests for uniform spatial grid broad-phase collision detection.
//
// Coverage:
//   1. Grid detection matches brute force on deterministic random scenes (sparse and dense).
//   2. Explicit collision pairs for Circle-Circle, AABB-AABB, and Circle-AABB.
//   3. Negative coordinates and cell-boundary crossings.
//   4. Large bodies spanning multiple grid cells.
//   5. Duplicate suppression: bodies sharing multiple cells emit candidate pairs at most once.
//   6. Zero bodies, one body, and separated sparse scenes emit zero contacts.
//   7. Removed bodies and stale/reused handles are safely excluded.
//   8. Static-static pairs are excluded.
//   9. Invalid cell sizes and non-finite coordinate inputs are safely rejected.
//  10. Repeated updates do not accumulate stale entries or leak memory.
//  11. Integration with SimulationController via BroadPhaseType::SpatialGrid.

#include "PhysicsEngine/Physics/SpatialGrid.h"
#include "PhysicsEngine/Physics/Collision.h"
#include "PhysicsEngine/Physics/Contact.h"
#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/SimulationController.h"
#include "PhysicsEngine/Physics/WorldConfig.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Math/Vec2.h"

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

Engine makeEngine(uint32_t capacity = 1024)
{
    WorldConfig cfg;
    cfg.maxBodies = capacity;
    cfg.gravity = {0.f, 0.f};
    return Engine{cfg};
}

BodyDesc makeCircle(Vec2 pos, float radius, float mass = 1.f)
{
    BodyDesc d;
    d.shapeType = ShapeType::Circle;
    d.position  = pos;
    d.radius    = radius;
    d.mass      = mass;
    return d;
}

BodyDesc makeAABB(Vec2 pos, float halfW, float halfH, float mass = 1.f)
{
    BodyDesc d;
    d.shapeType  = ShapeType::AABB;
    d.position   = pos;
    d.radius     = halfW;
    d.halfHeight = halfH;
    d.mass       = mass;
    return d;
}

using PairKey = std::pair<uint32_t, uint32_t>;

std::set<PairKey> extractCanonicalPairs(const std::vector<Contact>& contacts)
{
    std::set<PairKey> s;
    for (const auto& c : contacts) {
        uint32_t i = c.bodyA.index;
        uint32_t j = c.bodyB.index;
        s.insert({std::min(i, j), std::max(i, j)});
    }
    return s;
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// 1. Grid detection matches brute force on deterministic random scenes
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: matches brute force on sparse random scene", "[spatial_grid][equivalence]")
{
    Engine engine = makeEngine(256);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posDist(-40.f, 40.f);
    std::uniform_real_distribution<float> sizeDist(0.5f, 1.5f);
    std::uniform_int_distribution<int> shapeDist(0, 1);
    std::uniform_int_distribution<int> massDist(0, 4); // 20% static bodies

    for (int i = 0; i < 80; ++i) {
        const Vec2 pos{posDist(rng), posDist(rng)};
        const float mass = (massDist(rng) == 0) ? 0.f : 1.f;
        if (shapeDist(rng) == 0) {
            (void)engine.addBody(makeCircle(pos, sizeDist(rng), mass));
        } else {
            (void)engine.addBody(makeAABB(pos, sizeDist(rng), sizeDist(rng), mass));
        }
    }

    std::vector<Contact> contactsBF;
    detectCollisionsBruteForce(engine, contactsBF);

    SpatialGrid grid(3.0f);
    std::vector<Contact> contactsGrid;
    grid.detectCollisions(engine, contactsGrid);

    // Both broad-phases must identify the exact same set of colliding body pairs
    auto pairsBF   = extractCanonicalPairs(contactsBF);
    auto pairsGrid = extractCanonicalPairs(contactsGrid);
    REQUIRE(pairsBF == pairsGrid);

    // Grid candidate pairs must be vastly fewer than O(N^2) brute force pairs
    const size_t totalPairs = (80 * 79) / 2; // 3160 pairs
    REQUIRE(grid.lastCandidatePairCount() < totalPairs);
}

TEST_CASE("SpatialGrid: matches brute force on dense clustered scene", "[spatial_grid][equivalence]")
{
    Engine engine = makeEngine(256);
    std::mt19937 rng(1337);
    // Tight domain [-4, 4] ensures many overlapping clusters
    std::uniform_real_distribution<float> posDist(-4.f, 4.f);
    std::uniform_real_distribution<float> sizeDist(0.4f, 0.8f);
    std::uniform_int_distribution<int> shapeDist(0, 1);

    for (int i = 0; i < 60; ++i) {
        const Vec2 pos{posDist(rng), posDist(rng)};
        if (shapeDist(rng) == 0) {
            (void)engine.addBody(makeCircle(pos, sizeDist(rng)));
        } else {
            (void)engine.addBody(makeAABB(pos, sizeDist(rng), sizeDist(rng)));
        }
    }

    std::vector<Contact> contactsBF;
    detectCollisionsBruteForce(engine, contactsBF);

    SpatialGrid grid(1.5f);
    std::vector<Contact> contactsGrid;
    grid.detectCollisions(engine, contactsGrid);

    auto pairsBF   = extractCanonicalPairs(contactsBF);
    auto pairsGrid = extractCanonicalPairs(contactsGrid);
    REQUIRE(pairsBF == pairsGrid);
    REQUIRE(pairsGrid.size() > 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Shape combinations: Circle-Circle, AABB-AABB, Circle-AABB
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: detects Circle-Circle, AABB-AABB, and Circle-AABB collisions", "[spatial_grid][shapes]")
{
    Engine engine = makeEngine(32);

    // Pair 1: Circle-Circle (at x = 0, y = 0)
    BodyHandle c1 = engine.addBody(makeCircle({0.f, 0.f}, 1.f));
    BodyHandle c2 = engine.addBody(makeCircle({1.5f, 0.f}, 1.f));

    // Pair 2: AABB-AABB (at x = 10, y = 10)
    BodyHandle b1 = engine.addBody(makeAABB({10.f, 10.f}, 1.f, 1.f));
    BodyHandle b2 = engine.addBody(makeAABB({11.5f, 10.f}, 1.f, 1.f));

    // Pair 3: Circle-AABB (at x = -10, y = -10)
    BodyHandle ca1 = engine.addBody(makeCircle({-10.f, -10.f}, 1.f));
    BodyHandle ca2 = engine.addBody(makeAABB({-8.5f, -10.f}, 1.f, 1.f));

    SpatialGrid grid(2.0f);
    auto contacts = grid.detectCollisions(engine);

    REQUIRE(contacts.size() == 3);
    auto pairs = extractCanonicalPairs(contacts);

    REQUIRE(pairs.count({std::min(c1.index, c2.index), std::max(c1.index, c2.index)}) == 1);
    REQUIRE(pairs.count({std::min(b1.index, b2.index), std::max(b1.index, b2.index)}) == 1);
    REQUIRE(pairs.count({std::min(ca1.index, ca2.index), std::max(ca1.index, ca2.index)}) == 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Negative coordinates and cell boundaries
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: negative coordinates and cell boundary crossings", "[spatial_grid][boundaries]")
{
    Engine engine = makeEngine(32);
    // Cell size = 2.0. Boundaries exist at ..., -4.0, -2.0, 0.0, 2.0, 4.0, ...
    SpatialGrid grid(2.0f);

    // Body A centered right on negative boundary x = -2.0, radius 0.6 (spans x in [-2.6, -1.4], cells -2 and -1)
    (void)engine.addBody(makeCircle({-2.0f, -2.0f}, 0.6f));
    // Body B at (-1.5, -2.0), radius 0.6 (spans x in [-2.1, -0.9], cells -2 and -1)
    // Distance = 0.5 < 1.2 -> penetrates by 0.7
    (void)engine.addBody(makeCircle({-1.5f, -2.0f}, 0.6f));

    auto contacts = grid.detectCollisions(engine);
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].penetration == Approx(0.7f));
    REQUIRE(contacts[0].normal.x == Approx(1.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Large bodies spanning multiple cells
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: large body spanning many cells detects all overlaps", "[spatial_grid][large]")
{
    Engine engine = makeEngine(64);
    // Large static ground AABB: center (0, -1), halfWidth = 20, halfHeight = 1
    // Spans X from -20 to +20 (width 40 m = 20 cells wide with cellSize 2.0)
    BodyHandle hFloor = engine.addBody(makeAABB({0.f, -1.f}, 20.f, 1.f, 0.f));

    // Add 5 circles scattered across the floor surface
    std::vector<BodyHandle> circles;
    for (float x : {-15.f, -8.f, 0.f, 7.f, 16.f}) {
        circles.push_back(engine.addBody(makeCircle({x, 0.5f}, 0.8f)));
    }

    SpatialGrid grid(2.0f);
    auto contacts = grid.detectCollisions(engine);

    // Each circle touches the floor: exactly 5 contacts
    REQUIRE(contacts.size() == 5);
    for (const auto& c : contacts) {
        REQUIRE((c.bodyA == hFloor || c.bodyB == hFloor));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Duplicate suppression: sharing multiple cells emits pair at most once
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: duplicate pairs spanning multiple cells are emitted at most once", "[spatial_grid][dedup]")
{
    Engine engine = makeEngine(32);
    // Two large AABBs (4x4 meters) centered at (0, 0) and (1, 1).
    // Both span cells [-2, 2] x [-2, 2] -> sharing at least 9 cells!
    (void)engine.addBody(makeAABB({0.f, 0.f}, 2.f, 2.f));
    (void)engine.addBody(makeAABB({1.f, 1.f}, 2.f, 2.f));

    SpatialGrid grid(1.0f); // 1-meter cells maximize cell overlap
    auto contacts = grid.detectCollisions(engine);

    // Exactly ONE contact emitted despite sharing multiple cells
    REQUIRE(contacts.size() == 1);
    REQUIRE(grid.lastCandidatePairCount() == 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Zero bodies, one body, and sparse scenes emit zero contacts
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: zero and one body scenes emit zero contacts", "[spatial_grid][empty]")
{
    SpatialGrid grid(2.0f);
    std::vector<Contact> contacts;

    // Zero bodies
    {
        Engine engine = makeEngine(16);
        grid.detectCollisions(engine, contacts);
        REQUIRE(contacts.empty());
        REQUIRE(grid.lastCandidatePairCount() == 0);
    }

    // One body
    {
        Engine engine = makeEngine(16);
        (void)engine.addBody(makeCircle({0.f, 0.f}, 1.f));
        grid.detectCollisions(engine, contacts);
        REQUIRE(contacts.empty());
        REQUIRE(grid.lastCandidatePairCount() == 0);
    }

    // Multiple widely separated bodies
    {
        Engine engine = makeEngine(16);
        (void)engine.addBody(makeCircle({-50.f, 0.f}, 1.f));
        (void)engine.addBody(makeCircle({0.f, 50.f}, 1.f));
        (void)engine.addBody(makeCircle({50.f, -50.f}, 1.f));
        grid.detectCollisions(engine, contacts);
        REQUIRE(contacts.empty());
        REQUIRE(grid.lastCandidatePairCount() == 0);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. Removed bodies and stale/reused handles
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: removed bodies and stale handles are safely excluded", "[spatial_grid][stale]")
{
    Engine engine = makeEngine(32);
    BodyHandle h1 = engine.addBody(makeCircle({0.f, 0.f}, 1.f));
    BodyHandle h2 = engine.addBody(makeCircle({1.f, 0.f}, 1.f));

    SpatialGrid grid(2.0f);
    REQUIRE(grid.detectCollisions(engine).size() == 1);

    // Remove h2
    engine.removeBody(h2);
    REQUIRE(grid.detectCollisions(engine).empty());

    // Add h3 into vacated slot with different position (no collision)
    BodyHandle h3 = engine.addBody(makeCircle({10.f, 0.f}, 1.f));
    REQUIRE(grid.detectCollisions(engine).empty());

    // Move h3 to overlap h1
    engine.setPosition(h3, {1.2f, 0.f});
    auto contacts = grid.detectCollisions(engine);
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].bodyA == h1);
    REQUIRE(contacts[0].bodyB == h3);
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. Static-static pair exclusion
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: static-static pairs are excluded from broad-phase and emission", "[spatial_grid][static]")
{
    Engine engine = makeEngine(16);
    // Two static overlapping bodies
    (void)engine.addBody(makeCircle({0.f, 0.f}, 1.f, 0.f));
    (void)engine.addBody(makeCircle({0.5f, 0.f}, 1.f, 0.f));

    SpatialGrid grid(2.0f);
    auto contacts = grid.detectCollisions(engine);
    REQUIRE(contacts.empty());
    REQUIRE(grid.lastCandidatePairCount() == 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// 9. Invalid cell sizes and invalid coordinates handling
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: invalid cell size and non-finite geometry are safely handled", "[spatial_grid][validation]")
{
    const float nanF = std::numeric_limits<float>::quiet_NaN();
    const float infF = std::numeric_limits<float>::infinity();

    // Invalid cell size fallback
    SpatialGrid grid(-5.f);
    REQUIRE(grid.cellSize() == SpatialGrid::kDefaultCellSize);

    grid.setCellSize(0.f);
    REQUIRE(grid.cellSize() == SpatialGrid::kDefaultCellSize);

    grid.setCellSize(nanF);
    REQUIRE(grid.cellSize() == SpatialGrid::kDefaultCellSize);

    grid.setCellSize(infF);
    REQUIRE(grid.cellSize() == SpatialGrid::kDefaultCellSize);

    // Valid cell size works
    grid.setCellSize(4.5f);
    REQUIRE(grid.cellSize() == 4.5f);

    // Non-finite coordinates in engine bodies
    Engine engine = makeEngine(16);
    (void)engine.addBody(makeCircle({nanF, 0.f}, 1.f));
    (void)engine.addBody(makeCircle({0.f, infF}, 1.f));
    (void)engine.addBody(makeCircle({0.f, 0.f}, 1.f));

    auto contacts = grid.detectCollisions(engine);
    REQUIRE(contacts.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// 10. Repeated updates without stale entries
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: repeated updates do not accumulate stale entries", "[spatial_grid][stability]")
{
    Engine engine = makeEngine(16);
    (void)engine.addBody(makeCircle({0.f, 0.f}, 1.f));
    (void)engine.addBody(makeCircle({1.2f, 0.f}, 1.f));

    SpatialGrid grid(2.0f);

    // 50 consecutive detect calls
    for (int step = 0; step < 50; ++step) {
        auto contacts = grid.detectCollisions(engine);
        REQUIRE(contacts.size() == 1);
        REQUIRE(grid.lastCandidatePairCount() == 1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 11. SimulationController integration via BroadPhaseType::SpatialGrid
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: SimulationController runs SpatialGrid broad-phase seamlessly", "[controller][spatial_grid]")
{
    Engine engine = makeEngine(32);
    // Two dynamic circles moving toward each other along X
    BodyHandle hA = engine.addBody(makeCircle({-1.f, 0.f}, 1.f, 1.f));
    BodyHandle hB = engine.addBody(makeCircle({1.f, 0.f}, 1.f, 1.f));
    engine.setVelocity(hA, {2.f, 0.f});
    engine.setVelocity(hB, {-2.f, 0.f});

    SimulationController controller;
    REQUIRE(controller.broadPhaseType() == BroadPhaseType::BruteForce);

    // Switch to SpatialGrid
    controller.setBroadPhaseType(BroadPhaseType::SpatialGrid);
    REQUIRE(controller.broadPhaseType() == BroadPhaseType::SpatialGrid);

    // Step once to detect collision and resolve bounce
    controller.step(engine);

    // Collided and bounced
    REQUIRE(controller.lastContacts().size() == 1);
    REQUIRE(engine.getVelocity(hA).x < 0.f);
    REQUIRE(engine.getVelocity(hB).x > 0.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// 12. Oversized floor spanning 2000m detects collisions at extreme edges
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: oversized floor spanning 2000m detects collisions at extreme edges", "[spatial_grid][oversized]")
{
    Engine engine = makeEngine(64);
    // Huge static floor: center (0, -2), halfWidth = 1000, halfHeight = 2 -> top surface y = 0
    // Total width = 2000 meters. Spans 1000 cells of size 2.0m!
    BodyHandle hFloor = engine.addBody(makeAABB({0.f, -2.f}, 1000.f, 2.f, 0.f));

    // Circle near far left edge: x = -900, y = 0.5 (radius 0.8 -> overlaps floor)
    BodyHandle cLeft = engine.addBody(makeCircle({-900.f, 0.5f}, 0.8f));
    // Circle near center: x = 0, y = 0.5
    BodyHandle cMid = engine.addBody(makeCircle({0.f, 0.5f}, 0.8f));
    // Circle near far right edge: x = +900, y = 0.5
    BodyHandle cRight = engine.addBody(makeCircle({900.f, 0.5f}, 0.8f));
    // Circle far right above floor: x = +900, y = 5.0 (separated, no collision)
    (void)engine.addBody(makeCircle({900.f, 5.0f}, 0.8f));

    SpatialGrid grid(2.0f);
    std::vector<Contact> contactsGrid;
    grid.detectCollisions(engine, contactsGrid);

    std::vector<Contact> contactsBF;
    detectCollisionsBruteForce(engine, contactsBF);

    // Exact canonical pair-set equivalence required
    auto pairsGrid = extractCanonicalPairs(contactsGrid);
    auto pairsBF   = extractCanonicalPairs(contactsBF);
    REQUIRE(pairsGrid == pairsBF);
    REQUIRE(pairsGrid.size() == 3);

    REQUIRE(pairsGrid.count({std::min(hFloor.index, cLeft.index), std::max(hFloor.index, cLeft.index)}) == 1);
    REQUIRE(pairsGrid.count({std::min(hFloor.index, cMid.index), std::max(hFloor.index, cMid.index)}) == 1);
    REQUIRE(pairsGrid.count({std::min(hFloor.index, cRight.index), std::max(hFloor.index, cRight.index)}) == 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// 13. Large shapes overlapping bodies whose centers are many cells away
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: large shape overlapping bodies with distant centers", "[spatial_grid][oversized]")
{
    Engine engine = makeEngine(64);
    // Large dynamic box: center (0, 0), halfWidth = 100, halfHeight = 50
    // Spans 100 cells wide by 50 cells high
    (void)engine.addBody(makeAABB({0.f, 0.f}, 100.f, 50.f, 10.f));

    // Dynamic circles inside / on edges:
    (void)engine.addBody(makeCircle({95.f, 45.f}, 2.f));   // near top-right corner
    (void)engine.addBody(makeCircle({-95.f, -45.f}, 2.f)); // near bottom-left corner
    (void)engine.addBody(makeCircle({0.f, 0.f}, 2.f));      // at center
    // Outside
    (void)engine.addBody(makeCircle({120.f, 0.f}, 2.f));             // 18m separated from right edge

    SpatialGrid grid(2.0f);
    auto contactsGrid = grid.detectCollisions(engine);
    auto contactsBF   = detectCollisionsBruteForce(engine);

    REQUIRE(extractCanonicalPairs(contactsGrid) == extractCanonicalPairs(contactsBF));
    REQUIRE(contactsGrid.size() == 3);
}

// ─────────────────────────────────────────────────────────────────────────────
// 14. Coordinate boundaries and out-of-range rejection
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: coordinate boundary safety and out-of-range rejection", "[spatial_grid][bounds]")
{
    Engine engine = makeEngine(32);
    // Near large supported boundary: x = 500,000 meters
    BodyHandle hNear1 = engine.addBody(makeCircle({500000.f, 0.f}, 1.f));
    BodyHandle hNear2 = engine.addBody(makeCircle({500001.5f, 0.f}, 1.f));

    // Out-of-supported-range body: x = 20,000,000 meters (> kMaxCoord 1e7)
    (void)engine.addBody(makeCircle({20000000.f, 0.f}, 1.f));

    SpatialGrid grid(2.0f);
    auto contactsGrid = grid.detectCollisions(engine);
    auto contactsBF   = detectCollisionsBruteForce(engine);

    // Near boundary bodies collide cleanly
    REQUIRE(extractCanonicalPairs(contactsGrid) == extractCanonicalPairs(contactsBF));
    REQUIRE(contactsGrid.size() == 1);
    REQUIRE(contactsGrid[0].bodyA == hNear1);
    REQUIRE(contactsGrid[0].bodyB == hNear2);
}

// ─────────────────────────────────────────────────────────────────────────────
// 15. Differential fuzz equivalence over multiple seeds
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: differential fuzz equivalence over multiple deterministic scenes", "[spatial_grid][fuzz]")
{
    const uint32_t seeds[] = { 101, 202, 303, 404, 505 };

    for (uint32_t seed : seeds) {
        Engine engine = makeEngine(128);
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> posDist(-60.f, 60.f);
        std::uniform_real_distribution<float> sizeDist(0.2f, 3.0f);
        std::uniform_int_distribution<int> shapeDist(0, 1);
        std::uniform_int_distribution<int> staticDist(0, 5); // ~17% static

        // Add 60 bodies
        for (int i = 0; i < 60; ++i) {
            const Vec2 pos{posDist(rng), posDist(rng)};
            const float mass = (staticDist(rng) == 0) ? 0.f : 1.f;
            if (shapeDist(rng) == 0) {
                (void)engine.addBody(makeCircle(pos, sizeDist(rng), mass));
            } else {
                (void)engine.addBody(makeAABB(pos, sizeDist(rng), sizeDist(rng), mass));
            }
        }

        // Add 1 large body (span > 30 cells) to stress oversized-body logic
        (void)engine.addBody(makeAABB({0.f, 0.f}, 40.f, 15.f, 0.f));

        SpatialGrid grid(2.5f);
        auto contactsGrid = grid.detectCollisions(engine);
        auto contactsBF   = detectCollisionsBruteForce(engine);

        // Strict pair-set equivalence required on every seed
        auto pairsGrid = extractCanonicalPairs(contactsGrid);
        auto pairsBF   = extractCanonicalPairs(contactsBF);
        REQUIRE(pairsGrid == pairsBF);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 16. Cell key and pair key packing injectivity and collision immunity
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SpatialGrid: cell key and pair key packing are strictly injective", "[spatial_grid][keys]")
{
    // Test that coordinates across all 4 quadrants map to distinct 64-bit keys
    const int32_t coords[] = { -1000, -1, 0, 1, 1000 };
    std::set<uint64_t> cellKeys;

    for (int32_t cx : coords) {
        for (int32_t cy : coords) {
            uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) |
                            static_cast<uint64_t>(static_cast<uint32_t>(cy));
            REQUIRE(cellKeys.count(key) == 0); // No collision
            cellKeys.insert(key);

            // Reconstructed values match exactly
            int32_t recX = static_cast<int32_t>(static_cast<uint32_t>(key >> 32));
            int32_t recY = static_cast<int32_t>(static_cast<uint32_t>(key & 0xFFFFFFFFULL));
            REQUIRE(recX == cx);
            REQUIRE(recY == cy);
        }
    }

    // Test pair key packing
    std::set<uint64_t> pairKeys;
    for (uint32_t a = 0; a < 10; ++a) {
        for (uint32_t b = a + 1; b < 10; ++b) {
            uint64_t pKey = (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
            REQUIRE(pairKeys.count(pKey) == 0);
            pairKeys.insert(pKey);

            uint32_t recA = static_cast<uint32_t>(pKey >> 32);
            uint32_t recB = static_cast<uint32_t>(pKey & 0xFFFFFFFFULL);
            REQUIRE(recA == a);
            REQUIRE(recB == b);
        }
    }
}
