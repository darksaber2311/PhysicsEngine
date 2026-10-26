#pragma once

// Vec2.h -- Custom 2D vector type for PhysicsEngine2D.
//
// Design decisions:
//   - Header-only: all methods are short enough to inline everywhere without LTO.
//   - No external dependencies (no GLM, no Eigen, no standard math beyond <cmath>).
//   - float precision: sufficient for a real-time rigid-body simulation at
//     human-scale distances. double would double memory bandwidth in SoA loops.
//   - normalized() returns (0,0) for near-zero vectors rather than NaN/inf.
//     Threshold is lengthSq < 1e-24f (i.e. length < 1e-12f), which is well
//     below any physically meaningful vector in a physics simulation.
//   - operator==  uses exact float comparison -- intended for tests with
//     known-exact values. Use Catch2's Approx() for numerically-computed results.

#include <cmath>    // std::sqrt, std::abs
#include <cstdint>  // not used here, but conventionally pulled in by math headers

namespace PhysicsEngine {

struct Vec2 {
    float x = 0.f;
    float y = 0.f;

    // ── Constructors ──────────────────────────────────────────────────────────

    constexpr Vec2() noexcept = default;
    constexpr Vec2(float x_, float y_) noexcept : x(x_), y(y_) {}

    // ── Arithmetic (non-mutating) ─────────────────────────────────────────────

    [[nodiscard]] constexpr Vec2 operator+(Vec2 rhs) const noexcept {
        return {x + rhs.x, y + rhs.y};
    }

    [[nodiscard]] constexpr Vec2 operator-(Vec2 rhs) const noexcept {
        return {x - rhs.x, y - rhs.y};
    }

    [[nodiscard]] constexpr Vec2 operator*(float s) const noexcept {
        return {x * s, y * s};
    }

    // Division by scalar. Caller is responsible for s != 0.
    // Dividing by zero produces inf/nan — no silent masking.
    [[nodiscard]] constexpr Vec2 operator/(float s) const noexcept {
        return {x / s, y / s};
    }

    // Unary negation.
    [[nodiscard]] constexpr Vec2 operator-() const noexcept {
        return {-x, -y};
    }

    // ── Compound assignment ───────────────────────────────────────────────────

    constexpr Vec2& operator+=(Vec2 rhs) noexcept {
        x += rhs.x; y += rhs.y; return *this;
    }

    constexpr Vec2& operator-=(Vec2 rhs) noexcept {
        x -= rhs.x; y -= rhs.y; return *this;
    }

    constexpr Vec2& operator*=(float s) noexcept {
        x *= s; y *= s; return *this;
    }

    constexpr Vec2& operator/=(float s) noexcept {
        x /= s; y /= s; return *this;
    }

    // ── Equality (exact float comparison — use only for known-exact values) ───

    [[nodiscard]] constexpr bool operator==(Vec2 rhs) const noexcept {
        return x == rhs.x && y == rhs.y;
    }

    [[nodiscard]] constexpr bool operator!=(Vec2 rhs) const noexcept {
        return !(*this == rhs);
    }

    // ── Geometric operations ─────────────────────────────────────────────────

    // Dot product: |a||b|cos(theta).
    // For unit vectors: positive = same direction, 0 = perpendicular, negative = opposite.
    [[nodiscard]] constexpr float dot(Vec2 rhs) const noexcept {
        return x * rhs.x + y * rhs.y;
    }

    // Squared length -- avoids the sqrt; useful when only relative magnitudes matter.
    [[nodiscard]] constexpr float lengthSq() const noexcept {
        return x * x + y * y;
    }

    // Euclidean length.
    [[nodiscard]] float length() const noexcept {
        return std::sqrt(lengthSq());
    }

    // Unit vector in the same direction.
    // Returns (0, 0) if the input is zero or near-zero (lengthSq < 1e-24f),
    // which prevents division by zero and avoids NaN propagation.
    //
    // Why 1e-24f? It equals (1e-12f)^2. Any vector shorter than 1 picometer
    // (about the radius of an atomic nucleus) is treated as zero. This is
    // far below any physically meaningful scale for a rigid-body simulation
    // but far above floating-point underflow (~1.2e-38f for float).
    [[nodiscard]] Vec2 normalized() const noexcept {
        constexpr float kEpsilonSq = 1e-24f;
        const float lsq = lengthSq();
        if (lsq < kEpsilonSq) {
            return {0.f, 0.f};
        }
        return *this / std::sqrt(lsq);
    }

    // Perpendicular vector: rotate 90 degrees counter-clockwise.
    // perp({1,0}) == {0,1}. Length is preserved.
    [[nodiscard]] constexpr Vec2 perp() const noexcept {
        return {-y, x};
    }

    // ── Static helpers ────────────────────────────────────────────────────────

    [[nodiscard]] static constexpr float dot(Vec2 a, Vec2 b) noexcept {
        return a.dot(b);
    }

    [[nodiscard]] static float distance(Vec2 a, Vec2 b) noexcept {
        return (a - b).length();
    }

    [[nodiscard]] static constexpr float distanceSq(Vec2 a, Vec2 b) noexcept {
        return (a - b).lengthSq();
    }
};

// ── Free-function operators ───────────────────────────────────────────────────

// Left-scalar multiply: 2.f * v  (same as v * 2.f, but symmetric API)
[[nodiscard]] constexpr Vec2 operator*(float s, Vec2 v) noexcept {
    return v * s;
}

} // namespace PhysicsEngine
