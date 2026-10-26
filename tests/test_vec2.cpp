// test_vec2.cpp -- Comprehensive Catch2 tests for Vec2.
//
// Coverage:
//   - Construction (default and parameterized)
//   - All arithmetic operators (non-mutating and compound)
//   - Unary negation
//   - Dot product (orthogonal, parallel, arbitrary)
//   - lengthSq and length (3-4-5 right triangle, unit vectors)
//   - normalized(): normal vector, zero vector, near-zero vector, large values
//   - perp(): 90-degree CCW, length preservation
//   - Static helpers: dot, distance, distanceSq
//   - Left-scalar multiply symmetry
//   - Equality / inequality operators
//   - Edge cases: negative components, very large values

#include "PhysicsEngine/Math/Vec2.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <limits>

using PhysicsEngine::Vec2;
using Catch::Approx;

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Vec2: default constructor produces zero vector", "[vec2][ctor]")
{
    Vec2 v;
    REQUIRE(v.x == 0.f);
    REQUIRE(v.y == 0.f);
}

TEST_CASE("Vec2: parameterized constructor stores components", "[vec2][ctor]")
{
    Vec2 v{3.f, -4.f};
    REQUIRE(v.x == 3.f);
    REQUIRE(v.y == -4.f);
}

TEST_CASE("Vec2: negative components are stored correctly", "[vec2][ctor]")
{
    Vec2 v{-100.f, -200.f};
    REQUIRE(v.x == -100.f);
    REQUIRE(v.y == -200.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Equality
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Vec2: equality and inequality operators", "[vec2][equality]")
{
    Vec2 a{1.f, 2.f};
    Vec2 b{1.f, 2.f};
    Vec2 c{1.f, 3.f};
    Vec2 d{0.f, 2.f};

    REQUIRE(a == b);
    REQUIRE_FALSE(a == c);
    REQUIRE_FALSE(a == d);
    REQUIRE(a != c);
    REQUIRE(a != d);
}

// ─────────────────────────────────────────────────────────────────────────────
// Arithmetic — non-mutating
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Vec2: operator+ adds components", "[vec2][arithmetic]")
{
    Vec2 a{1.f, 2.f};
    Vec2 b{3.f, -1.f};
    Vec2 result = a + b;
    REQUIRE(result.x == Approx(4.f));
    REQUIRE(result.y == Approx(1.f));
    // Original vectors are unchanged
    REQUIRE(a.x == 1.f);
    REQUIRE(b.x == 3.f);
}

TEST_CASE("Vec2: operator- subtracts components", "[vec2][arithmetic]")
{
    Vec2 a{5.f, 3.f};
    Vec2 b{2.f, 4.f};
    Vec2 result = a - b;
    REQUIRE(result.x == Approx(3.f));
    REQUIRE(result.y == Approx(-1.f));
}

TEST_CASE("Vec2: operator* scales by scalar", "[vec2][arithmetic]")
{
    Vec2 v{2.f, -3.f};
    Vec2 result = v * 4.f;
    REQUIRE(result.x == Approx(8.f));
    REQUIRE(result.y == Approx(-12.f));
}

TEST_CASE("Vec2: operator* by zero produces zero vector", "[vec2][arithmetic]")
{
    Vec2 v{100.f, -200.f};
    Vec2 result = v * 0.f;
    REQUIRE(result.x == 0.f);
    REQUIRE(result.y == 0.f);
}

TEST_CASE("Vec2: operator/ divides by scalar", "[vec2][arithmetic]")
{
    Vec2 v{6.f, -9.f};
    Vec2 result = v / 3.f;
    REQUIRE(result.x == Approx(2.f));
    REQUIRE(result.y == Approx(-3.f));
}

TEST_CASE("Vec2: unary negation flips signs", "[vec2][arithmetic]")
{
    Vec2 v{3.f, -4.f};
    Vec2 neg = -v;
    REQUIRE(neg.x == -3.f);
    REQUIRE(neg.y == 4.f);

    // Double negation restores original
    REQUIRE((-neg).x == 3.f);
    REQUIRE((-neg).y == -4.f);
}

TEST_CASE("Vec2: left-scalar multiply is symmetric", "[vec2][arithmetic]")
{
    Vec2 v{2.f, 5.f};
    Vec2 r1 = v * 3.f;
    Vec2 r2 = 3.f * v;
    REQUIRE(r1.x == Approx(r2.x));
    REQUIRE(r1.y == Approx(r2.y));
}

// ─────────────────────────────────────────────────────────────────────────────
// Compound assignment
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Vec2: compound assignment operators modify in place", "[vec2][arithmetic]")
{
    Vec2 v{10.f, 20.f};

    v += Vec2{1.f, 2.f};
    REQUIRE(v.x == Approx(11.f));
    REQUIRE(v.y == Approx(22.f));

    v -= Vec2{1.f, 2.f};
    REQUIRE(v.x == Approx(10.f));
    REQUIRE(v.y == Approx(20.f));

    v *= 2.f;
    REQUIRE(v.x == Approx(20.f));
    REQUIRE(v.y == Approx(40.f));

    v /= 4.f;
    REQUIRE(v.x == Approx(5.f));
    REQUIRE(v.y == Approx(10.f));
}

TEST_CASE("Vec2: compound assignment returns reference to self", "[vec2][arithmetic]")
{
    Vec2 v{1.f, 2.f};
    Vec2& ref = (v += Vec2{3.f, 4.f});
    // ref should be the same object as v
    REQUIRE(&ref == &v);
    REQUIRE(v.x == Approx(4.f));
    REQUIRE(v.y == Approx(6.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Dot product
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Vec2: dot product of orthogonal vectors is zero", "[vec2][dot]")
{
    Vec2 a{1.f, 0.f};
    Vec2 b{0.f, 1.f};
    REQUIRE(a.dot(b) == Approx(0.f));
    REQUIRE(Vec2::dot(a, b) == Approx(0.f));
}

TEST_CASE("Vec2: dot product of parallel vectors equals product of lengths", "[vec2][dot]")
{
    Vec2 a{3.f, 0.f};
    Vec2 b{5.f, 0.f};
    REQUIRE(a.dot(b) == Approx(15.f));
}

TEST_CASE("Vec2: dot product of anti-parallel vectors is negative", "[vec2][dot]")
{
    Vec2 a{1.f, 0.f};
    Vec2 b{-1.f, 0.f};
    REQUIRE(a.dot(b) == Approx(-1.f));
}

TEST_CASE("Vec2: dot product is commutative", "[vec2][dot]")
{
    Vec2 a{3.f, 7.f};
    Vec2 b{-2.f, 5.f};
    REQUIRE(a.dot(b) == Approx(b.dot(a)));
    REQUIRE(a.dot(b) == Approx(-6.f + 35.f));  // 3*(-2) + 7*5 = 29
}

// ─────────────────────────────────────────────────────────────────────────────
// Length
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Vec2: lengthSq and length on zero vector", "[vec2][length]")
{
    Vec2 v{};
    REQUIRE(v.lengthSq() == Approx(0.f));
    REQUIRE(v.length()   == Approx(0.f));
}

TEST_CASE("Vec2: lengthSq and length on 3-4-5 right triangle", "[vec2][length]")
{
    Vec2 v{3.f, 4.f};
    REQUIRE(v.lengthSq() == Approx(25.f));
    REQUIRE(v.length()   == Approx(5.f));
}

TEST_CASE("Vec2: lengthSq and length on negative components", "[vec2][length]")
{
    Vec2 v{-3.f, -4.f};
    REQUIRE(v.lengthSq() == Approx(25.f));
    REQUIRE(v.length()   == Approx(5.f));
}

TEST_CASE("Vec2: length of unit axis vectors is 1", "[vec2][length]")
{
    REQUIRE(Vec2{1.f, 0.f}.length() == Approx(1.f));
    REQUIRE(Vec2{0.f, 1.f}.length() == Approx(1.f));
    REQUIRE(Vec2{-1.f, 0.f}.length() == Approx(1.f));
}

TEST_CASE("Vec2: length with large components stays finite", "[vec2][length]")
{
    // 1e18f squared = 1e36f, which is within float's range (~3.4e38).
    // Using 1e20f would overflow: (1e20)^2 = 1e40 > float_max, giving inf.
    Vec2 v{1e18f, 0.f};
    REQUIRE(v.length() == Approx(1e18f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Normalization
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Vec2: normalized produces unit vector", "[vec2][normalize]")
{
    Vec2 v{3.f, 4.f};
    Vec2 n = v.normalized();
    REQUIRE(n.length() == Approx(1.f));
    REQUIRE(n.x == Approx(3.f / 5.f));
    REQUIRE(n.y == Approx(4.f / 5.f));
}

TEST_CASE("Vec2: normalized of unit vector is itself", "[vec2][normalize]")
{
    Vec2 v{1.f, 0.f};
    Vec2 n = v.normalized();
    REQUIRE(n.x == Approx(1.f));
    REQUIRE(n.y == Approx(0.f));
}

TEST_CASE("Vec2: normalized of negative-component vector points in correct direction", "[vec2][normalize]")
{
    Vec2 v{-3.f, -4.f};
    Vec2 n = v.normalized();
    REQUIRE(n.length() == Approx(1.f));
    REQUIRE(n.x == Approx(-3.f / 5.f));
    REQUIRE(n.y == Approx(-4.f / 5.f));
}

TEST_CASE("Vec2: normalized of zero vector returns zero vector (no NaN)", "[vec2][normalize][edge]")
{
    Vec2 zero{};
    Vec2 n = zero.normalized();
    REQUIRE(n.x == 0.f);
    REQUIRE(n.y == 0.f);
    // Explicit NaN/inf check
    REQUIRE(std::isfinite(n.x));
    REQUIRE(std::isfinite(n.y));
}

TEST_CASE("Vec2: normalized of near-zero vector returns zero vector (no NaN)", "[vec2][normalize][edge]")
{
    // lengthSq = 2e-25, which is below the 1e-24 threshold
    Vec2 tiny{1e-13f, 0.f};    // lengthSq = 1e-26 -- below threshold
    Vec2 n = tiny.normalized();
    REQUIRE(n.x == 0.f);
    REQUIRE(n.y == 0.f);
    REQUIRE(std::isfinite(n.x));
    REQUIRE(std::isfinite(n.y));
}

TEST_CASE("Vec2: normalized of barely-above-threshold vector is a unit vector", "[vec2][normalize][edge]")
{
    // lengthSq = 4e-24, which is above the 1e-24 threshold
    Vec2 v{2e-12f, 0.f};
    Vec2 n = v.normalized();
    REQUIRE(n.length() == Approx(1.f).margin(1e-5));
}

TEST_CASE("Vec2: normalized of large-component vector is unit vector", "[vec2][normalize]")
{
    Vec2 v{1e15f, 1e15f};
    Vec2 n = v.normalized();
    REQUIRE(n.length() == Approx(1.f).margin(1e-5));
    REQUIRE(std::isfinite(n.x));
    REQUIRE(std::isfinite(n.y));
}

// ─────────────────────────────────────────────────────────────────────────────
// Perpendicular
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Vec2: perp rotates 90 degrees counter-clockwise", "[vec2][perp]")
{
    // perp({1,0}) should be {0,1}
    Vec2 x{1.f, 0.f};
    Vec2 p = x.perp();
    REQUIRE(p.x == Approx(0.f));
    REQUIRE(p.y == Approx(1.f));

    // perp({0,1}) should be {-1,0}
    Vec2 y{0.f, 1.f};
    Vec2 q = y.perp();
    REQUIRE(q.x == Approx(-1.f));
    REQUIRE(q.y == Approx(0.f));
}

TEST_CASE("Vec2: perp preserves length", "[vec2][perp]")
{
    Vec2 v{3.f, 4.f};
    REQUIRE(v.perp().length() == Approx(v.length()));
}

TEST_CASE("Vec2: perp is orthogonal to original", "[vec2][perp]")
{
    Vec2 v{3.f, 7.f};
    REQUIRE(v.dot(v.perp()) == Approx(0.f));
}

TEST_CASE("Vec2: double perp returns original vector negated... wait no", "[vec2][perp]")
{
    // perp(perp(v)) = -v  (rotating 180 degrees total returns -v)
    // Actually perp({x,y}) = {-y,x}, perp({-y,x}) = {-x,-y} = -v
    Vec2 v{3.f, -4.f};
    Vec2 pp = v.perp().perp();
    REQUIRE(pp.x == Approx(-v.x));
    REQUIRE(pp.y == Approx(-v.y));
}

// ─────────────────────────────────────────────────────────────────────────────
// Static helpers
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Vec2: static dot product matches member dot product", "[vec2][helpers]")
{
    Vec2 a{2.f, 3.f};
    Vec2 b{4.f, -1.f};
    REQUIRE(Vec2::dot(a, b) == Approx(a.dot(b)));
    REQUIRE(Vec2::dot(a, b) == Approx(5.f));  // 2*4 + 3*(-1) = 5
}

TEST_CASE("Vec2: distanceSq between two points", "[vec2][helpers]")
{
    Vec2 a{0.f, 0.f};
    Vec2 b{3.f, 4.f};
    REQUIRE(Vec2::distanceSq(a, b) == Approx(25.f));
    REQUIRE(Vec2::distanceSq(b, a) == Approx(25.f));  // symmetric
}

TEST_CASE("Vec2: distance between two points", "[vec2][helpers]")
{
    Vec2 a{0.f, 0.f};
    Vec2 b{3.f, 4.f};
    REQUIRE(Vec2::distance(a, b) == Approx(5.f));
}

TEST_CASE("Vec2: distance from point to itself is zero", "[vec2][helpers]")
{
    Vec2 a{7.f, -3.f};
    REQUIRE(Vec2::distance(a, a) == Approx(0.f));
    REQUIRE(Vec2::distanceSq(a, a) == Approx(0.f));
}
