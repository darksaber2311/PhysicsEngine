// test_engine.cpp -- Comprehensive Catch2 tests for the SoA PhysicsEngine.
//
// Coverage:
//   - Engine construction with valid and edge-case configs
//   - addBody: returns valid handle, body count increases
//   - getPosition/setPosition, getVelocity/setVelocity round-trips
//   - Mass validation: static bodies, zero mass, negative mass
//   - Restitution clamping
//   - Shape storage (Circle and AABB)
//   - Capacity: fill to maxBodies; next add returns null handle
//   - removeBody: bodyCount decreases, handle becomes invalid
//   - Stale handle: old handle fails after remove
//   - Generation safety: re-add to same slot; old handle cannot access new body
//   - Swap-and-pop correctness: remove a middle body; all other handles valid
//   - Repeated add/remove cycles: no corruption
//   - clear(): all handles invalid, bodyCount == 0, re-add works
//   - Invalid/null handle access: all getters return documented safe defaults
//   - addForce: accumulates for dynamic, ignored for static
//   - clearForces: resets to zero
//   - Raw SoA pointer access
//   - Null handle isNull() / isValidHandle()

#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Physics/WorldConfig.h"
#include "PhysicsEngine/Math/Vec2.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

using namespace PhysicsEngine;
using Catch::Approx;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static BodyDesc makeCircle(Vec2 pos, float mass = 1.f, float radius = 1.f,
                           Vec2 vel = {}, float restitution = 0.5f)
{
    BodyDesc d;
    d.position    = pos;
    d.velocity    = vel;
    d.mass        = mass;
    d.radius      = radius;
    d.restitution = restitution;
    d.shapeType   = ShapeType::Circle;
    return d;
}

static BodyDesc makeAABB(Vec2 pos, float hw, float hh,
                         float mass = 1.f, float restitution = 0.5f)
{
    BodyDesc d;
    d.position    = pos;
    d.mass        = mass;
    d.radius      = hw;
    d.halfHeight  = hh;
    d.restitution = restitution;
    d.shapeType   = ShapeType::AABB;
    return d;
}

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: default construction", "[engine][ctor]")
{
    Engine e;
    REQUIRE(e.bodyCount()    == 0);
    REQUIRE(e.bodyCapacity() == 1024);  // WorldConfig default
}

TEST_CASE("Engine: custom WorldConfig capacity", "[engine][ctor]")
{
    WorldConfig cfg;
    cfg.maxBodies = 64;
    Engine e{cfg};
    REQUIRE(e.bodyCapacity() == 64);
    REQUIRE(e.bodyCount()    == 0);
}

TEST_CASE("Engine: zero maxBodies is clamped to 1", "[engine][ctor]")
{
    WorldConfig cfg;
    cfg.maxBodies = 0;
    Engine e{cfg};
    REQUIRE(e.bodyCapacity() >= 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// addBody — basic
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: addBody returns non-null handle and increments bodyCount", "[engine][add]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({1.f, 2.f}));
    REQUIRE_FALSE(h.isNull());
    REQUIRE(e.isValidHandle(h));
    REQUIRE(e.bodyCount() == 1);
}

TEST_CASE("Engine: addBody multiple bodies", "[engine][add]")
{
    Engine e;
    BodyHandle h0 = e.addBody(makeCircle({0.f, 0.f}));
    BodyHandle h1 = e.addBody(makeCircle({1.f, 1.f}));
    BodyHandle h2 = e.addBody(makeCircle({2.f, 2.f}));

    REQUIRE(e.bodyCount() == 3);
    REQUIRE(e.isValidHandle(h0));
    REQUIRE(e.isValidHandle(h1));
    REQUIRE(e.isValidHandle(h2));
    // Handles must be distinct
    REQUIRE(h0 != h1);
    REQUIRE(h1 != h2);
    REQUIRE(h0 != h2);
}

// ─────────────────────────────────────────────────────────────────────────────
// Position / velocity round-trips
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: position round-trip via addBody + getPosition", "[engine][position]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({3.f, -7.f}));
    Vec2 pos = e.getPosition(h);
    REQUIRE(pos.x == Approx(3.f));
    REQUIRE(pos.y == Approx(-7.f));
}

TEST_CASE("Engine: setPosition and getPosition", "[engine][position]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({0.f, 0.f}));
    e.setPosition(h, {5.f, -10.f});
    Vec2 pos = e.getPosition(h);
    REQUIRE(pos.x == Approx(5.f));
    REQUIRE(pos.y == Approx(-10.f));
}

TEST_CASE("Engine: velocity round-trip via addBody + getVelocity", "[engine][velocity]")
{
    Engine e;
    BodyDesc d = makeCircle({0.f, 0.f}, 1.f, 1.f, {3.f, 4.f});
    BodyHandle h = e.addBody(d);
    Vec2 vel = e.getVelocity(h);
    REQUIRE(vel.x == Approx(3.f));
    REQUIRE(vel.y == Approx(4.f));
}

TEST_CASE("Engine: setVelocity and getVelocity", "[engine][velocity]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({0.f, 0.f}));
    e.setVelocity(h, {-2.f, 6.f});
    Vec2 vel = e.getVelocity(h);
    REQUIRE(vel.x == Approx(-2.f));
    REQUIRE(vel.y == Approx(6.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Mass and static bodies
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: dynamic body has positive inverse mass", "[engine][mass]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({}, 2.f));   // mass = 2 kg
    REQUIRE(e.getInverseMass(h) == Approx(0.5f));
    REQUIRE_FALSE(e.isStatic(h));
}

TEST_CASE("Engine: zero mass produces static body", "[engine][mass]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({}, 0.f));
    REQUIRE(e.getInverseMass(h) == 0.f);
    REQUIRE(e.isStatic(h));
}

TEST_CASE("Engine: negative mass is treated as static", "[engine][mass]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({}, -5.f));
    REQUIRE(e.getInverseMass(h) == 0.f);
    REQUIRE(e.isStatic(h));
}

// ─────────────────────────────────────────────────────────────────────────────
// Restitution clamping
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: restitution in [0,1] is stored as-is", "[engine][restitution]")
{
    Engine e;
    BodyDesc d = makeCircle({}, 1.f, 1.f, {}, 0.7f);
    BodyHandle h = e.addBody(d);
    REQUIRE(e.getRestitution(h) == Approx(0.7f));
}

TEST_CASE("Engine: restitution above 1 is clamped to 1", "[engine][restitution]")
{
    Engine e;
    BodyDesc d = makeCircle({}, 1.f, 1.f, {}, 1.5f);
    BodyHandle h = e.addBody(d);
    REQUIRE(e.getRestitution(h) == Approx(1.f));
}

TEST_CASE("Engine: restitution below 0 is clamped to 0", "[engine][restitution]")
{
    Engine e;
    BodyDesc d = makeCircle({}, 1.f, 1.f, {}, -0.5f);
    BodyHandle h = e.addBody(d);
    REQUIRE(e.getRestitution(h) == Approx(0.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Shape storage
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: circle shape is stored correctly", "[engine][shape]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({}, 1.f, 2.5f));
    REQUIRE(e.getShapeType(h) == ShapeType::Circle);
    REQUIRE(e.getRadius(h)    == Approx(2.5f));
    REQUIRE(e.getHalfHeight(h) == Approx(0.f));  // not used for circles
}

TEST_CASE("Engine: AABB shape is stored correctly", "[engine][shape]")
{
    Engine e;
    BodyHandle h = e.addBody(makeAABB({}, 3.f, 1.5f));
    REQUIRE(e.getShapeType(h)  == ShapeType::AABB);
    REQUIRE(e.getRadius(h)     == Approx(3.f));   // half-width stored in radius slot
    REQUIRE(e.getHalfHeight(h) == Approx(1.5f));
}

TEST_CASE("Engine: invalid shape dimensions are sanitised", "[engine][shape]")
{
    Engine e;
    BodyDesc d = makeCircle({});
    d.radius = -1.f;  // invalid
    BodyHandle h = e.addBody(d);
    REQUIRE(e.getRadius(h) == Approx(1.f));  // clamped to 1.0 default
}

// ─────────────────────────────────────────────────────────────────────────────
// Capacity exhaustion
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: filling to capacity - all handles valid", "[engine][capacity]")
{
    constexpr uint32_t CAP = 8;
    WorldConfig cfg;
    cfg.maxBodies = CAP;
    Engine e{cfg};

    std::vector<BodyHandle> handles;
    handles.reserve(CAP);
    for (uint32_t i = 0; i < CAP; ++i) {
        BodyHandle h = e.addBody(makeCircle({static_cast<float>(i), 0.f}));
        REQUIRE_FALSE(h.isNull());
        handles.push_back(h);
    }
    REQUIRE(e.bodyCount() == CAP);

    // All handles must still be valid
    for (auto h : handles) {
        REQUIRE(e.isValidHandle(h));
    }
}

TEST_CASE("Engine: adding body beyond capacity returns null handle", "[engine][capacity]")
{
    WorldConfig cfg;
    cfg.maxBodies = 4;
    Engine e{cfg};

    for (uint32_t i = 0; i < 4; ++i) {
        (void)e.addBody(makeCircle({}));
    }
    REQUIRE(e.bodyCount() == 4);

    BodyHandle overflow = e.addBody(makeCircle({}));
    REQUIRE(overflow.isNull());
    REQUIRE_FALSE(e.isValidHandle(overflow));
    REQUIRE(e.bodyCount() == 4);  // count unchanged
}

// ─────────────────────────────────────────────────────────────────────────────
// removeBody — basic
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: removeBody decrements bodyCount", "[engine][remove]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({}));
    REQUIRE(e.bodyCount() == 1);
    e.removeBody(h);
    REQUIRE(e.bodyCount() == 0);
}

TEST_CASE("Engine: handle is invalid after removeBody", "[engine][remove]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({}));
    e.removeBody(h);
    REQUIRE_FALSE(e.isValidHandle(h));
}

TEST_CASE("Engine: removing a body twice is a no-op (idempotent)", "[engine][remove]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({}));
    e.removeBody(h);
    REQUIRE(e.bodyCount() == 0);
    e.removeBody(h);  // second remove — must not crash or corrupt state
    REQUIRE(e.bodyCount() == 0);
}

TEST_CASE("Engine: removing null handle is a no-op", "[engine][remove]")
{
    Engine e;
    (void)e.addBody(makeCircle({}));
    e.removeBody(BodyHandle::null());
    REQUIRE(e.bodyCount() == 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Stale handle and generation safety
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: stale handle cannot access a re-added body in same slot", "[engine][generation]")
{
    WorldConfig cfg;
    cfg.maxBodies = 2;
    Engine e{cfg};

    BodyHandle old = e.addBody(makeCircle({1.f, 1.f}));
    e.removeBody(old);

    // Re-add a new body (may reuse the same handle-table slot)
    BodyHandle fresh = e.addBody(makeCircle({99.f, 99.f}));
    REQUIRE(e.isValidHandle(fresh));

    // The old handle must NOT be valid regardless of slot reuse
    REQUIRE_FALSE(e.isValidHandle(old));

    // The old handle must NOT see the new body's position
    // getPosition on invalid handle returns (0,0), NOT the fresh body's position
    Vec2 posFromOld = e.getPosition(old);
    REQUIRE(posFromOld.x == 0.f);
    REQUIRE(posFromOld.y == 0.f);
}

TEST_CASE("Engine: generation increments prevent cross-lifetime access", "[engine][generation]")
{
    Engine e;
    BodyHandle h1 = e.addBody(makeCircle({1.f, 0.f}));
    uint32_t gen1 = h1.generation;

    e.removeBody(h1);
    BodyHandle h2 = e.addBody(makeCircle({2.f, 0.f}));

    // If the handle-table slot was reused, the generation must have incremented
    if (h2.index == h1.index) {
        REQUIRE(h2.generation != gen1);
    }

    REQUIRE_FALSE(e.isValidHandle(h1));
    REQUIRE(e.isValidHandle(h2));
}

// ─────────────────────────────────────────────────────────────────────────────
// Swap-and-pop correctness
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: removing middle body leaves other handles valid", "[engine][swap-pop]")
{
    Engine e;
    BodyHandle hA = e.addBody(makeCircle({1.f, 0.f}));
    BodyHandle hB = e.addBody(makeCircle({2.f, 0.f}));
    BodyHandle hC = e.addBody(makeCircle({3.f, 0.f}));

    // Remove the middle body (B).  C will be swapped into B's SoA slot.
    e.removeBody(hB);

    REQUIRE(e.bodyCount() == 2);
    REQUIRE(e.isValidHandle(hA));
    REQUIRE_FALSE(e.isValidHandle(hB));
    REQUIRE(e.isValidHandle(hC));

    // Data integrity: A and C retain their original positions
    REQUIRE(e.getPosition(hA).x == Approx(1.f));
    REQUIRE(e.getPosition(hC).x == Approx(3.f));
}

TEST_CASE("Engine: removing first body leaves last body accessible", "[engine][swap-pop]")
{
    Engine e;
    BodyHandle hA = e.addBody(makeCircle({10.f, 0.f}));
    BodyHandle hB = e.addBody(makeCircle({20.f, 0.f}));

    e.removeBody(hA);  // A is first; B swaps into slot 0

    REQUIRE(e.bodyCount() == 1);
    REQUIRE_FALSE(e.isValidHandle(hA));
    REQUIRE(e.isValidHandle(hB));
    REQUIRE(e.getPosition(hB).x == Approx(20.f));
}

TEST_CASE("Engine: removing last body (no swap needed)", "[engine][swap-pop]")
{
    Engine e;
    BodyHandle hA = e.addBody(makeCircle({10.f, 0.f}));
    BodyHandle hB = e.addBody(makeCircle({20.f, 0.f}));

    e.removeBody(hB);  // B is last; no swap, just decrement

    REQUIRE(e.bodyCount() == 1);
    REQUIRE(e.isValidHandle(hA));
    REQUIRE_FALSE(e.isValidHandle(hB));
    REQUIRE(e.getPosition(hA).x == Approx(10.f));
}

TEST_CASE("Engine: multiple removals maintain data integrity", "[engine][swap-pop]")
{
    Engine e;
    BodyHandle handles[5];
    for (int i = 0; i < 5; ++i) {
        handles[i] = e.addBody(makeCircle({static_cast<float>(i * 10), 0.f}));
    }

    // Remove bodies 1 and 3 (middle bodies)
    e.removeBody(handles[1]);
    e.removeBody(handles[3]);

    REQUIRE(e.bodyCount() == 3);
    REQUIRE(e.isValidHandle(handles[0]));
    REQUIRE_FALSE(e.isValidHandle(handles[1]));
    REQUIRE(e.isValidHandle(handles[2]));
    REQUIRE_FALSE(e.isValidHandle(handles[3]));
    REQUIRE(e.isValidHandle(handles[4]));

    REQUIRE(e.getPosition(handles[0]).x == Approx(0.f));
    REQUIRE(e.getPosition(handles[2]).x == Approx(20.f));
    REQUIRE(e.getPosition(handles[4]).x == Approx(40.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Repeated add/remove cycles
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: repeated add/remove cycles do not corrupt state", "[engine][stress]")
{
    WorldConfig cfg;
    cfg.maxBodies = 16;
    Engine e{cfg};

    // Cycle: add 8 bodies, remove all, repeat 10 times
    for (int cycle = 0; cycle < 10; ++cycle) {
        std::vector<BodyHandle> handles;
        for (int i = 0; i < 8; ++i) {
            handles.push_back(e.addBody(makeCircle({static_cast<float>(i), 0.f})));
        }
        REQUIRE(e.bodyCount() == 8);
        for (auto h : handles) {
            e.removeBody(h);
        }
        REQUIRE(e.bodyCount() == 0);
    }

    // After 10 cycles the engine must still accept bodies
    BodyHandle h = e.addBody(makeCircle({0.f, 0.f}));
    REQUIRE_FALSE(h.isNull());
    REQUIRE(e.isValidHandle(h));
}

// ─────────────────────────────────────────────────────────────────────────────
// Slot reuse after removal
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: slot is reusable after removal", "[engine][reuse]")
{
    WorldConfig cfg;
    cfg.maxBodies = 1;  // single-slot engine
    Engine e{cfg};

    BodyHandle h1 = e.addBody(makeCircle({1.f, 1.f}));
    REQUIRE_FALSE(h1.isNull());
    REQUIRE(e.bodyCount() == 1);

    e.removeBody(h1);
    REQUIRE(e.bodyCount() == 0);

    // The one slot is now available again
    BodyHandle h2 = e.addBody(makeCircle({2.f, 2.f}));
    REQUIRE_FALSE(h2.isNull());
    REQUIRE(e.bodyCount() == 1);
    REQUIRE(e.isValidHandle(h2));
    REQUIRE_FALSE(e.isValidHandle(h1));  // old handle must not work
    REQUIRE(e.getPosition(h2).x == Approx(2.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// clear()
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: clear() sets bodyCount to zero", "[engine][clear]")
{
    Engine e;
    (void)e.addBody(makeCircle({}));
    (void)e.addBody(makeCircle({}));
    e.clear();
    REQUIRE(e.bodyCount() == 0);
}

TEST_CASE("Engine: all handles invalid after clear()", "[engine][clear]")
{
    Engine e;
    BodyHandle h0 = e.addBody(makeCircle({}));
    BodyHandle h1 = e.addBody(makeCircle({}));
    e.clear();
    REQUIRE_FALSE(e.isValidHandle(h0));
    REQUIRE_FALSE(e.isValidHandle(h1));
}

TEST_CASE("Engine: addBody works normally after clear()", "[engine][clear]")
{
    Engine e;
    for (int i = 0; i < 10; ++i) (void)e.addBody(makeCircle({}));
    e.clear();

    BodyHandle h = e.addBody(makeCircle({5.f, 5.f}));
    REQUIRE_FALSE(h.isNull());
    REQUIRE(e.isValidHandle(h));
    REQUIRE(e.bodyCount() == 1);
    REQUIRE(e.getPosition(h).x == Approx(5.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Invalid handle access — safe defaults
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: null handle isNull and not valid", "[engine][null-handle]")
{
    Engine e;
    BodyHandle null = BodyHandle::null();
    REQUIRE(null.isNull());
    REQUIRE_FALSE(e.isValidHandle(null));
}

TEST_CASE("Engine: getters on invalid handle return safe defaults", "[engine][null-handle]")
{
    Engine e;
    BodyHandle inv = BodyHandle::null();

    REQUIRE(e.getPosition(inv)    == Vec2{});
    REQUIRE(e.getVelocity(inv)    == Vec2{});
    REQUIRE(e.getInverseMass(inv) == 0.f);   // static-body default
    REQUIRE(e.isStatic(inv)       == true);  // conservative default
    REQUIRE(e.getRestitution(inv) == 0.f);
    REQUIRE(e.getRadius(inv)      == 0.f);
    REQUIRE(e.getHalfHeight(inv)  == 0.f);
    REQUIRE(e.getShapeType(inv)   == ShapeType::Circle);
}

TEST_CASE("Engine: setters on invalid handle are no-ops", "[engine][null-handle]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({1.f, 1.f}));

    BodyHandle inv = BodyHandle::null();
    e.setPosition(inv, {99.f, 99.f});
    e.setVelocity(inv, {99.f, 99.f});

    // The valid body h is unaffected
    REQUIRE(e.getPosition(h).x == Approx(1.f));
    REQUIRE(e.getVelocity(h)   == Vec2{});
}

// ─────────────────────────────────────────────────────────────────────────────
// Force accumulation
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: addForce accumulates on dynamic body", "[engine][force]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({}, 2.f));  // dynamic

    e.addForce(h, {10.f, 0.f});
    e.addForce(h, {5.f, -3.f});

    const Vec2* forces = e.forcesData();
    uint32_t slot = 0;  // first body always at SoA slot 0 in an empty engine
    REQUIRE(forces[slot].x == Approx(15.f));
    REQUIRE(forces[slot].y == Approx(-3.f));
}

TEST_CASE("Engine: addForce is no-op on static body", "[engine][force]")
{
    Engine e;
    BodyHandle h = e.addBody(makeCircle({}, 0.f));  // static (mass=0)

    e.addForce(h, {999.f, 999.f});

    const Vec2* forces = e.forcesData();
    REQUIRE(forces[0].x == 0.f);
    REQUIRE(forces[0].y == 0.f);
}

TEST_CASE("Engine: clearForces resets all forces to zero", "[engine][force]")
{
    Engine e;
    BodyHandle h0 = e.addBody(makeCircle({}, 1.f));
    BodyHandle h1 = e.addBody(makeCircle({}, 1.f));

    e.addForce(h0, {5.f, 5.f});
    e.addForce(h1, {-3.f, 2.f});
    e.clearForces();

    const Vec2* forces = e.forcesData();
    REQUIRE(forces[0].x == 0.f);
    REQUIRE(forces[0].y == 0.f);
    REQUIRE(forces[1].x == 0.f);
    REQUIRE(forces[1].y == 0.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Raw SoA data access
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: positionsData pointer is consistent with getPosition", "[engine][soa]")
{
    Engine e;
    BodyHandle h0 = e.addBody(makeCircle({3.f, 4.f}));
    BodyHandle h1 = e.addBody(makeCircle({-1.f, 2.f}));

    const Vec2* pos = e.positionsData();
    // Both bodies are at consecutive SoA slots [0] and [1]
    REQUIRE(pos[0].x == Approx(e.getPosition(h0).x));
    REQUIRE(pos[0].y == Approx(e.getPosition(h0).y));
    REQUIRE(pos[1].x == Approx(e.getPosition(h1).x));
    REQUIRE(pos[1].y == Approx(e.getPosition(h1).y));
}

TEST_CASE("Engine: bodyCount() reflects the active SoA range", "[engine][soa]")
{
    Engine e;
    (void)e.addBody(makeCircle({}));
    (void)e.addBody(makeCircle({}));
    REQUIRE(e.bodyCount() == 2);

    // The integration loop can safely iterate [0, bodyCount())
    const Vec2* pos = e.positionsData();
    for (uint32_t i = 0; i < e.bodyCount(); ++i) {
        REQUIRE(std::isfinite(pos[i].x));
        REQUIRE(std::isfinite(pos[i].y));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// WorldConfig access
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Engine: config() returns the WorldConfig used at construction", "[engine][config]")
{
    WorldConfig cfg;
    cfg.maxBodies = 32;
    cfg.gravity   = {0.f, -20.f};
    cfg.fixedDt   = 1.f / 60.f;

    Engine e{cfg};
    REQUIRE(e.config().maxBodies == 32);
    REQUIRE(e.config().gravity.y == Approx(-20.f));
    REQUIRE(e.config().fixedDt   == Approx(1.f / 60.f));
}
