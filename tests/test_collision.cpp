// test_collision.cpp -- Catch2 tests for narrow-phase collision detection and pair generation.
//
// Coverage:
//   - Circle-Circle: overlapping, separated, tangent, coincident centers, unequal radii
//   - AABB-AABB: overlapping, separated, touching edge/corner, one contained in another
//   - Circle-AABB: overlapping, separated, tangent, circle center inside box
//   - Handle-based dispatch: valid handles, invalid/stale handles, self-collision rejection
//   - Brute-force pair generator:
//       * Zero and one body scenes (no pairs)
//       * Duplicate pair prevention (only unordered pairs (i, j) with i < j)
//       * Static-static pairs omitted
//       * Static-dynamic pairs detected
//       * Mixed Circle and AABB scenes
//   - Normal orientation (always points A -> B) and nonnegative penetration (> 0)

#include "PhysicsEngine/Physics/Collision.h"
#include "PhysicsEngine/Physics/Contact.h"
#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/WorldConfig.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Math/Vec2.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {

BodyDesc makeCircleDesc(Vec2 pos, float radius, float mass = 1.f)
{
    BodyDesc d;
    d.shapeType = ShapeType::Circle;
    d.position  = pos;
    d.radius    = radius;
    d.mass      = mass;
    return d;
}

BodyDesc makeAABBDesc(Vec2 pos, float halfWidth, float halfHeight, float mass = 1.f)
{
    BodyDesc d;
    d.shapeType  = ShapeType::AABB;
    d.position   = pos;
    d.radius     = halfWidth;
    d.halfHeight = halfHeight;
    d.mass       = mass;
    return d;
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Circle-Circle Tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Collision: overlapping circles produce valid contact", "[collision][circle]")
{
    Contact contact;
    // Circle A: pos (0, 0), radius 2. Circle B: pos (3, 0), radius 2.
    // Distance = 3, sum of radii = 4, penetration = 1.
    bool hit = testCircleCircle({0.f, 0.f}, 2.f, {3.f, 0.f}, 2.f, contact);

    REQUIRE(hit);
    REQUIRE(contact.penetration == Approx(1.f));
    // Normal points from A to B: (+1, 0)
    REQUIRE(contact.normal.x == Approx(1.f));
    REQUIRE(contact.normal.y == Approx(0.f));
    // Contact point is between surfaces: x = 0 + 1 * (2 - 0.5) = 1.5
    REQUIRE(contact.point.x == Approx(1.5f));
    REQUIRE(contact.point.y == Approx(0.f));
}

TEST_CASE("Collision: separated circles do not collide", "[collision][circle]")
{
    Contact contact;
    // Circle A: (0, 0), r=1. Circle B: (5, 0), r=1. Distance = 5 > 2.
    bool hit = testCircleCircle({0.f, 0.f}, 1.f, {5.f, 0.f}, 1.f, contact);
    REQUIRE_FALSE(hit);
}

TEST_CASE("Collision: tangent circles touching at boundary do not collide", "[collision][circle]")
{
    Contact contact;
    // Circle A: (0, 0), r=2. Circle B: (5, 0), r=3. Distance = 5 == 2 + 3.
    bool hit = testCircleCircle({0.f, 0.f}, 2.f, {5.f, 0.f}, 3.f, contact);
    REQUIRE_FALSE(hit);
}

TEST_CASE("Collision: coincident circle centers choose deterministic normal", "[collision][circle]")
{
    Contact contact;
    // Circle A: (2, 3), r=2. Circle B: (2, 3), r=4. Distance = 0.
    bool hit = testCircleCircle({2.f, 3.f}, 2.f, {2.f, 3.f}, 4.f, contact);

    REQUIRE(hit);
    // Penetration = rA + rB = 6.0
    REQUIRE(contact.penetration == Approx(6.f));
    // Deterministic unit normal: (0, 1)
    REQUIRE(contact.normal.x == Approx(0.f));
    REQUIRE(contact.normal.y == Approx(1.f));
    REQUIRE(std::isfinite(contact.normal.x));
    REQUIRE(std::isfinite(contact.normal.y));
}

TEST_CASE("Collision: unequal circle radii overlapping diagonally", "[collision][circle]")
{
    Contact contact;
    // A: (0, 0), r=3. B: (2, 2), r=1.
    // Distance = sqrt(8) ≈ 2.8284. Sum = 4.0. Penetration ≈ 1.1716.
    bool hit = testCircleCircle({0.f, 0.f}, 3.f, {2.f, 2.f}, 1.f, contact);

    REQUIRE(hit);
    const float dist = std::sqrt(8.f);
    REQUIRE(contact.penetration == Approx(4.f - dist));
    // Normal along (2, 2) normalized
    REQUIRE(contact.normal.x == Approx(std::sqrt(0.5f)));
    REQUIRE(contact.normal.y == Approx(std::sqrt(0.5f)));
}

// ─────────────────────────────────────────────────────────────────────────────
// AABB-AABB Tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Collision: overlapping AABBs select minimum penetration axis", "[collision][aabb]")
{
    Contact contact;
    // Box A: center (0, 0), halfExtents (2, 2) -> bounds [-2, 2] x [-2, 2]
    // Box B: center (3, 0.5), halfExtents (2, 2) -> bounds [1, 5] x [-1.5, 2.5]
    // Overlap X: (2 + 2) - |3 - 0| = 4 - 3 = 1.0
    // Overlap Y: (2 + 2) - |0.5 - 0| = 4 - 0.5 = 3.5
    // Minimum axis is X. Normal points from A to B: (+1, 0)
    bool hit = testAABBAABB({0.f, 0.f}, {2.f, 2.f}, {3.f, 0.5f}, {2.f, 2.f}, contact);

    REQUIRE(hit);
    REQUIRE(contact.penetration == Approx(1.f));
    REQUIRE(contact.normal.x == Approx(1.f));
    REQUIRE(contact.normal.y == Approx(0.f));
    // Contact X should be on interface x = 2.0
    REQUIRE(contact.point.x == Approx(2.f));
}

TEST_CASE("Collision: overlapping AABBs with minimum Y penetration", "[collision][aabb]")
{
    Contact contact;
    // Box A: center (0, 0), halfExtents (3, 2)
    // Box B: center (0.5, 3), halfExtents (3, 2)
    // Overlap X: (3 + 3) - 0.5 = 5.5
    // Overlap Y: (2 + 2) - 3.0 = 1.0
    // Minimum axis is Y. Normal points from A to B: (0, 1)
    bool hit = testAABBAABB({0.f, 0.f}, {3.f, 2.f}, {0.5f, 3.f}, {3.f, 2.f}, contact);

    REQUIRE(hit);
    REQUIRE(contact.penetration == Approx(1.f));
    REQUIRE(contact.normal.x == Approx(0.f));
    REQUIRE(contact.normal.y == Approx(1.f));
    REQUIRE(contact.point.y == Approx(2.f));
}

TEST_CASE("Collision: separated AABBs do not collide", "[collision][aabb]")
{
    Contact contact;
    // Separated on X
    bool hit = testAABBAABB({0.f, 0.f}, {1.f, 1.f}, {3.f, 0.f}, {1.f, 1.f}, contact);
    REQUIRE_FALSE(hit);

    // Separated on Y
    hit = testAABBAABB({0.f, 0.f}, {1.f, 1.f}, {0.f, 5.f}, {1.f, 1.f}, contact);
    REQUIRE_FALSE(hit);
}

TEST_CASE("Collision: AABBs touching at edge or corner do not collide", "[collision][aabb]")
{
    Contact contact;
    // Touching on edge: Box A [-1, 1], Box B [1, 3] -> overlapX = 0
    bool hitEdge = testAABBAABB({0.f, 0.f}, {1.f, 1.f}, {2.f, 0.f}, {1.f, 1.f}, contact);
    REQUIRE_FALSE(hitEdge);

    // Touching on corner: Box A [-1, 1] x [-1, 1], Box B [1, 3] x [1, 3]
    bool hitCorner = testAABBAABB({0.f, 0.f}, {1.f, 1.f}, {2.f, 2.f}, {1.f, 1.f}, contact);
    REQUIRE_FALSE(hitCorner);
}

TEST_CASE("Collision: one AABB contained inside another", "[collision][aabb]")
{
    Contact contact;
    // Big Box A: center (0, 0), halfExtents (5, 5)
    // Small Box B: center (1, 0), halfExtents (1, 1)
    // Overlap X: (5 + 1) - 1 = 5
    // Overlap Y: (5 + 1) - 0 = 6
    // Minimum axis is X: penetration = 5, normal (+1, 0)
    bool hit = testAABBAABB({0.f, 0.f}, {5.f, 5.f}, {1.f, 0.f}, {1.f, 1.f}, contact);

    REQUIRE(hit);
    REQUIRE(contact.penetration == Approx(5.f));
    REQUIRE(contact.normal.x == Approx(1.f));
    REQUIRE(contact.normal.y == Approx(0.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Circle-AABB Tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Collision: circle overlapping AABB edge", "[collision][circle-aabb]")
{
    Contact contact;
    // Circle A: center (0, 0), radius 2
    // AABB B: center (3, 0), halfExtents (2, 2) -> bounds [1, 5] x [-2, 2]
    // Closest point on AABB to circle is (1, 0).
    // Distance from circle center (0, 0) to (1, 0) is 1.0. Radius = 2.0.
    // Penetration = 2 - 1 = 1.0.
    // Normal points circle A -> AABB B: (+1, 0).
    bool hit = testCircleAABB({0.f, 0.f}, 2.f, {3.f, 0.f}, {2.f, 2.f}, contact);

    REQUIRE(hit);
    REQUIRE(contact.penetration == Approx(1.f));
    REQUIRE(contact.normal.x == Approx(1.f));
    REQUIRE(contact.normal.y == Approx(0.f));
    REQUIRE(contact.point.x == Approx(1.f));
    REQUIRE(contact.point.y == Approx(0.f));
}

TEST_CASE("Collision: circle center inside AABB", "[collision][circle-aabb]")
{
    Contact contact;
    // Circle center at (0, 0), radius 1
    // AABB center at (0, 0), halfExtents (4, 2)
    // Closest face is top/bottom (dy = 2 vs dx = 4).
    // Normal points toward closest edge.
    bool hit = testCircleAABB({0.f, 0.f}, 1.f, {0.f, 0.f}, {4.f, 2.f}, contact);

    REQUIRE(hit);
    REQUIRE(contact.penetration > 0.f);
    REQUIRE(std::isfinite(contact.normal.x));
    REQUIRE(std::isfinite(contact.normal.y));
}

TEST_CASE("Collision: circle separated from AABB", "[collision][circle-aabb]")
{
    Contact contact;
    bool hit = testCircleAABB({0.f, 0.f}, 1.f, {10.f, 0.f}, {2.f, 2.f}, contact);
    REQUIRE_FALSE(hit);
}

// ─────────────────────────────────────────────────────────────────────────────
// Handle-based Dispatch & Validation
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Collision: testCollision validates handles and rejects self-collision", "[collision][dispatcher]")
{
    Engine engine;
    BodyHandle h1 = engine.addBody(makeCircleDesc({0.f, 0.f}, 1.f));
    BodyHandle h2 = engine.addBody(makeCircleDesc({1.f, 0.f}, 1.f));

    Contact contact;

    // Valid collision
    REQUIRE(testCollision(engine, h1, h2, contact));
    REQUIRE(contact.bodyA == h1);
    REQUIRE(contact.bodyB == h2);

    // Self-collision rejected
    REQUIRE_FALSE(testCollision(engine, h1, h1, contact));

    // Null handle rejected
    REQUIRE_FALSE(testCollision(engine, h1, BodyHandle::null(), contact));
    REQUIRE_FALSE(testCollision(engine, BodyHandle::null(), h2, contact));

    // Stale handle after removal rejected
    engine.removeBody(h2);
    REQUIRE_FALSE(testCollision(engine, h1, h2, contact));
}

// ─────────────────────────────────────────────────────────────────────────────
// Brute-force Pair Generator Tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Collision: brute-force scan on zero and one body scenes emits no contacts", "[collision][bruteforce]")
{
    Engine engine;
    std::vector<Contact> contacts;

    // Zero bodies
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.empty());

    // One body
    (void)engine.addBody(makeCircleDesc({0.f, 0.f}, 2.f));
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.empty());
}

TEST_CASE("Collision: brute-force scan prevents duplicate pairs", "[collision][bruteforce]")
{
    Engine engine;
    // Two overlapping circles
    BodyHandle hA = engine.addBody(makeCircleDesc({0.f, 0.f}, 2.f));
    BodyHandle hB = engine.addBody(makeCircleDesc({1.f, 0.f}, 2.f));

    auto contacts = detectCollisionsBruteForce(engine);

    // Exactly 1 contact pair (A, B) — never duplicate (B, A)
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].bodyA == hA);
    REQUIRE(contacts[0].bodyB == hB);
    REQUIRE(contacts[0].penetration > 0.f);
}

TEST_CASE("Collision: static-static pairs are omitted from collision scan", "[collision][bruteforce]")
{
    Engine engine;
    // Two overlapping static circles (mass = 0)
    BodyHandle s1 = engine.addBody(makeCircleDesc({0.f, 0.f}, 2.f, 0.f));
    BodyHandle s2 = engine.addBody(makeCircleDesc({1.f, 0.f}, 2.f, 0.f));
    REQUIRE(engine.isValidHandle(s1));
    REQUIRE(engine.isValidHandle(s2));

    auto contacts = detectCollisionsBruteForce(engine);

    // Omitted by policy
    REQUIRE(contacts.empty());

    // Add a dynamic circle overlapping s1
    BodyHandle d1 = engine.addBody(makeCircleDesc({0.5f, 0.f}, 1.f, 1.f));
    REQUIRE(engine.isValidHandle(d1));
    contacts = detectCollisionsBruteForce(engine);

    // Should detect dynamic-static collisions, but still no static-static
    REQUIRE_FALSE(contacts.empty());
    for (const auto& c : contacts) {
        bool bothStatic = engine.isStatic(c.bodyA) && engine.isStatic(c.bodyB);
        REQUIRE_FALSE(bothStatic);
    }
}

TEST_CASE("Collision: mixed Circle and AABB scene detects all overlaps cleanly", "[collision][bruteforce]")
{
    Engine engine;
    // Circle at (0, 0), r = 2
    BodyHandle hCircle = engine.addBody(makeCircleDesc({0.f, 0.f}, 2.f));
    // AABB at (1.5, 0), hw = 1, hh = 1 (overlaps circle)
    BodyHandle hAABB1 = engine.addBody(makeAABBDesc({1.5f, 0.f}, 1.f, 1.f));
    // Separated AABB at (10, 10), hw = 1, hh = 1
    BodyHandle hAABB2 = engine.addBody(makeAABBDesc({10.f, 10.f}, 1.f, 1.f));
    REQUIRE(engine.isValidHandle(hAABB2));

    auto contacts = detectCollisionsBruteForce(engine);

    // Exactly one overlap: hCircle and hAABB1
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].bodyA == hCircle);
    REQUIRE(contacts[0].bodyB == hAABB1);
    REQUIRE(contacts[0].penetration > 0.f);
    REQUIRE(contacts[0].normal.x > 0.f); // Points circle -> AABB (+X)
}

// ─────────────────────────────────────────────────────────────────────────────
// Targeted Regression Tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Collision: contained AABB with various offsets computes exact MTD and normal", "[collision][aabb][containment]")
{
    Contact contact;
    const Vec2 bigPos{0.f, 0.f};
    const Vec2 bigExt{10.f, 10.f};
    const Vec2 smallExt{2.f, 2.f};

    // 1. Shifted +X: closest exit is through right face (+X)
    // Overlap X: (10 + 2) - 4 = 8. Overlap Y: (10 + 2) - 1 = 11.
    // Small box must move +X to exit -> normal (+1, 0), penetration = 8
    {
        bool hit = testAABBAABB(bigPos, bigExt, {4.f, 1.f}, smallExt, contact);
        REQUIRE(hit);
        REQUIRE(contact.penetration == Approx(8.f));
        REQUIRE(contact.normal.x == Approx(1.f));
        REQUIRE(contact.normal.y == Approx(0.f));
        REQUIRE(contact.point.x == Approx(10.f));
    }

    // 2. Shifted -X: closest exit is through left face (-X)
    // Overlap X: (10 + 2) - 4 = 8. Overlap Y: (10 + 2) - 1 = 11.
    // Small box must move -X to exit -> normal (-1, 0), penetration = 8
    {
        bool hit = testAABBAABB(bigPos, bigExt, {-4.f, 1.f}, smallExt, contact);
        REQUIRE(hit);
        REQUIRE(contact.penetration == Approx(8.f));
        REQUIRE(contact.normal.x == Approx(-1.f));
        REQUIRE(contact.normal.y == Approx(0.f));
        REQUIRE(contact.point.x == Approx(-10.f));
    }

    // 3. Shifted +Y: closest exit is through top face (+Y)
    // Overlap X: (10 + 2) - 1 = 11. Overlap Y: (10 + 2) - 5 = 7.
    // Small box must move +Y to exit -> normal (0, 1), penetration = 7
    {
        bool hit = testAABBAABB(bigPos, bigExt, {1.f, 5.f}, smallExt, contact);
        REQUIRE(hit);
        REQUIRE(contact.penetration == Approx(7.f));
        REQUIRE(contact.normal.x == Approx(0.f));
        REQUIRE(contact.normal.y == Approx(1.f));
        REQUIRE(contact.point.y == Approx(10.f));
    }

    // 4. Shifted -Y: closest exit is through bottom face (-Y)
    // Small box must move -Y to exit -> normal (0, -1), penetration = 7
    {
        bool hit = testAABBAABB(bigPos, bigExt, {1.f, -5.f}, smallExt, contact);
        REQUIRE(hit);
        REQUIRE(contact.penetration == Approx(7.f));
        REQUIRE(contact.normal.x == Approx(0.f));
        REQUIRE(contact.normal.y == Approx(-1.f));
        REQUIRE(contact.point.y == Approx(-10.f));
    }
}

TEST_CASE("Collision: circle-AABB in both body orders preserves separation normal and contact point", "[collision][circle-aabb][order]")
{
    Engine engine;

    // Subcase A: Circle outside AABB (above the top face)
    {
        BodyHandle hCircle = engine.addBody(makeCircleDesc({0.f, 3.f}, 2.f));
        BodyHandle hAABB   = engine.addBody(makeAABBDesc({0.f, 0.f}, 2.f, 2.f));

        Contact contactAB;
        Contact contactBA;

        // Order 1: Circle as bodyA, AABB as bodyB
        REQUIRE(testCollision(engine, hCircle, hAABB, contactAB));
        REQUIRE(contactAB.bodyA == hCircle);
        REQUIRE(contactAB.bodyB == hAABB);
        // Normal points from Circle to AABB: downward (0, -1)
        REQUIRE(contactAB.normal.x == Approx(0.f).margin(1e-5f));
        REQUIRE(contactAB.normal.y == Approx(-1.f));
        REQUIRE(contactAB.penetration == Approx(1.f));

        // Order 2: AABB as bodyA, Circle as bodyB
        REQUIRE(testCollision(engine, hAABB, hCircle, contactBA));
        REQUIRE(contactBA.bodyA == hAABB);
        REQUIRE(contactBA.bodyB == hCircle);
        // Normal points from AABB to Circle: upward (0, +1)
        REQUIRE(contactBA.normal.x == Approx(0.f).margin(1e-5f));
        REQUIRE(contactBA.normal.y == Approx(1.f));
        REQUIRE(contactBA.penetration == Approx(1.f));

        // Contact point must match between both queries
        REQUIRE(contactAB.point.x == Approx(contactBA.point.x));
        REQUIRE(contactAB.point.y == Approx(contactBA.point.y));

        engine.clear();
    }

    // Subcase B: Circle inside AABB (near top face)
    {
        BodyHandle hCircle = engine.addBody(makeCircleDesc({0.f, 1.5f}, 1.f));
        BodyHandle hAABB   = engine.addBody(makeAABBDesc({0.f, 0.f}, 3.f, 2.f));

        Contact contactAB;
        Contact contactBA;

        // Order 1: Circle as bodyA, AABB as bodyB
        REQUIRE(testCollision(engine, hCircle, hAABB, contactAB));
        // Solver pushes bodyA along -normal, bodyB along +normal.
        // For Circle to exit through top face (move +Y), normal must be (0, -1).
        REQUIRE(contactAB.normal.x == Approx(0.f).margin(1e-5f));
        REQUIRE(contactAB.normal.y == Approx(-1.f));
        REQUIRE(contactAB.penetration == Approx(1.5f));
        REQUIRE(contactAB.point.y == Approx(2.f)); // on top face

        // Order 2: AABB as bodyA, Circle as bodyB
        REQUIRE(testCollision(engine, hAABB, hCircle, contactBA));
        // Normal points from AABB to Circle: (0, 1)
        REQUIRE(contactBA.normal.x == Approx(0.f).margin(1e-5f));
        REQUIRE(contactBA.normal.y == Approx(1.f));
        REQUIRE(contactBA.penetration == Approx(1.5f));
        REQUIRE(contactBA.point.y == Approx(2.f));

        engine.clear();
    }
}

TEST_CASE("Collision: coincident and near-zero circle centers produce deterministic finite contacts", "[collision][circle][coincident]")
{
    Contact contact;

    // Exact coincidence
    {
        bool hit = testCircleCircle({1.f, 2.f}, 3.f, {1.f, 2.f}, 2.f, contact);
        REQUIRE(hit);
        REQUIRE(contact.penetration == Approx(5.f));
        REQUIRE(contact.normal.x == Approx(0.f));
        REQUIRE(contact.normal.y == Approx(1.f));
        REQUIRE(std::isfinite(contact.point.x));
        REQUIRE(std::isfinite(contact.point.y));
    }

    // Sub-epsilon distance (1e-8 < 1e-6)
    {
        bool hit = testCircleCircle({0.f, 0.f}, 1.f, {1e-8f, 0.f}, 1.f, contact);
        REQUIRE(hit);
        REQUIRE(contact.penetration == Approx(2.f));
        REQUIRE(contact.normal.x == Approx(0.f));
        REQUIRE(contact.normal.y == Approx(1.f));
    }

    // Above-epsilon small distance (1e-4 > 1e-6)
    {
        bool hit = testCircleCircle({0.f, 0.f}, 1.f, {1e-4f, 0.f}, 1.f, contact);
        REQUIRE(hit);
        REQUIRE(contact.penetration == Approx(2.f - 1e-4f));
        REQUIRE(contact.normal.x == Approx(1.f));
        REQUIRE(contact.normal.y == Approx(0.f));
    }
}

TEST_CASE("Collision: invalid dimensions and non-finite geometry rejected safely", "[collision][validation]")
{
    Contact contact;
    const float nanVal = std::numeric_limits<float>::quiet_NaN();
    const float infVal = std::numeric_limits<float>::infinity();

    // Circle-Circle
    REQUIRE_FALSE(testCircleCircle({0.f, 0.f}, 0.f, {1.f, 0.f}, 1.f, contact));
    REQUIRE_FALSE(testCircleCircle({0.f, 0.f}, -1.f, {1.f, 0.f}, 1.f, contact));
    REQUIRE_FALSE(testCircleCircle({0.f, 0.f}, nanVal, {1.f, 0.f}, 1.f, contact));
    REQUIRE_FALSE(testCircleCircle({0.f, 0.f}, infVal, {1.f, 0.f}, 1.f, contact));
    REQUIRE_FALSE(testCircleCircle({nanVal, 0.f}, 1.f, {1.f, 0.f}, 1.f, contact));
    REQUIRE_FALSE(testCircleCircle({0.f, infVal}, 1.f, {1.f, 0.f}, 1.f, contact));

    // AABB-AABB
    REQUIRE_FALSE(testAABBAABB({0.f, 0.f}, {0.f, 1.f}, {0.f, 0.f}, {1.f, 1.f}, contact));
    REQUIRE_FALSE(testAABBAABB({0.f, 0.f}, {-1.f, 1.f}, {0.f, 0.f}, {1.f, 1.f}, contact));
    REQUIRE_FALSE(testAABBAABB({0.f, 0.f}, {nanVal, 1.f}, {0.f, 0.f}, {1.f, 1.f}, contact));
    REQUIRE_FALSE(testAABBAABB({infVal, 0.f}, {1.f, 1.f}, {0.f, 0.f}, {1.f, 1.f}, contact));

    // Circle-AABB
    REQUIRE_FALSE(testCircleAABB({0.f, 0.f}, -1.f, {0.f, 0.f}, {1.f, 1.f}, contact));
    REQUIRE_FALSE(testCircleAABB({0.f, 0.f}, 1.f, {0.f, 0.f}, {0.f, 1.f}, contact));
    REQUIRE_FALSE(testCircleAABB({nanVal, 0.f}, 1.f, {0.f, 0.f}, {1.f, 1.f}, contact));
    REQUIRE_FALSE(testCircleAABB({0.f, 0.f}, 1.f, {infVal, 0.f}, {1.f, 1.f}, contact));
}

TEST_CASE("Collision: stale handles after slot removal and reuse rejected", "[collision][stale]")
{
    Engine engine;
    BodyHandle h1 = engine.addBody(makeCircleDesc({0.f, 0.f}, 2.f));
    BodyHandle h2 = engine.addBody(makeCircleDesc({1.f, 0.f}, 2.f));

    Contact contact;
    REQUIRE(testCollision(engine, h1, h2, contact));

    // Remove h2
    engine.removeBody(h2);
    REQUIRE_FALSE(testCollision(engine, h1, h2, contact));

    // Re-add a new body h3 into the same slot (reuses slot with incremented generation)
    BodyHandle h3 = engine.addBody(makeCircleDesc({1.f, 0.f}, 2.f));
    REQUIRE(engine.isValidHandle(h3));
    REQUIRE_FALSE(engine.isValidHandle(h2)); // h2 remains stale

    // Old handle h2 is rejected
    REQUIRE_FALSE(testCollision(engine, h1, h2, contact));

    // New handle h3 is valid
    REQUIRE(testCollision(engine, h1, h3, contact));

    // Pair generator contains h1 and h3, not h2
    auto contacts = detectCollisionsBruteForce(engine);
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].bodyA == h1);
    REQUIRE(contacts[0].bodyB == h3);
}

TEST_CASE("Collision: all emitted contacts have finite unit normals and positive penetration", "[collision][invariants]")
{
    Engine engine;
    // Add multiple overlapping shapes in a cluster
    (void)engine.addBody(makeCircleDesc({0.f, 0.f}, 2.f));
    (void)engine.addBody(makeCircleDesc({1.f, 0.5f}, 1.5f));
    (void)engine.addBody(makeAABBDesc({0.5f, 0.f}, 1.f, 1.f));
    (void)engine.addBody(makeAABBDesc({1.f, 0.2f}, 2.f, 1.5f));

    auto contacts = detectCollisionsBruteForce(engine);
    REQUIRE(contacts.size() > 0);

    for (const auto& c : contacts) {
        REQUIRE(std::isfinite(c.normal.x));
        REQUIRE(std::isfinite(c.normal.y));
        REQUIRE(c.normal.lengthSq() == Approx(1.f).margin(1e-5f));
        REQUIRE(std::isfinite(c.penetration));
        REQUIRE(c.penetration > 0.f);
        REQUIRE(std::isfinite(c.point.x));
        REQUIRE(std::isfinite(c.point.y));
    }
}
