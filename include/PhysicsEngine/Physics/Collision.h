#pragma once

// Collision.h -- Narrow-phase collision detection and reference pair generator.
//
// Conventions & Invariants:
//   - Collision Normal: Always points from bodyA toward bodyB.
//   - Penetration: Strictly positive (> 0.f). Touching / tangent boundaries
//     (penetration <= 0.f) do not generate contacts.
//   - Coincident Centers:
//     - For circles: normal defaults deterministically to (0, 1), avoiding 0/0 NaNs.
//     - For AABBs: normal defaults deterministically along the minimum axis.
//   - Pair Generation:
//     - Never emits self-pairs (i == j).
//     - Never emits duplicate unordered pairs: checks (i, j) with i < j.
//     - Omit static-static pairs: two static bodies cannot move, so resolving
//       contacts between them is physically unnecessary.
//   - Headless: Zero GLFW / OpenGL dependencies.

#include "PhysicsEngine/Math/Vec2.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Physics/Contact.h"
#include "PhysicsEngine/Physics/Engine.h"

#include <vector>

namespace PhysicsEngine {

// ── Geometric Narrowphase Tests ──────────────────────────────────────────────

/// Tests collision between two circles.
///
/// \param posA         Center of circle A.
/// \param radiusA      Radius of circle A (> 0).
/// \param posB         Center of circle B.
/// \param radiusB      Radius of circle B (> 0).
/// \param outContact   Populated with normal, penetration, and point if overlapping.
/// \param bodyA        Optional handle to assign to outContact.bodyA.
/// \param bodyB        Optional handle to assign to outContact.bodyB.
/// \return             True if circles penetrate (penetration > 0).
bool testCircleCircle(Vec2 posA, float radiusA,
                      Vec2 posB, float radiusB,
                      Contact& outContact,
                      BodyHandle bodyA = BodyHandle::null(),
                      BodyHandle bodyB = BodyHandle::null()) noexcept;

/// Tests collision between two axis-aligned bounding boxes (AABBs).
///
/// \param posA          Center of AABB A.
/// \param halfExtentsA  Half-width (x) and half-height (y) of box A (> 0).
/// \param posB          Center of AABB B.
/// \param halfExtentsB  Half-width (x) and half-height (y) of box B (> 0).
/// \param outContact    Populated with normal, penetration, and point if overlapping.
/// \param bodyA         Optional handle to assign to outContact.bodyA.
/// \param bodyB         Optional handle to assign to outContact.bodyB.
/// \return              True if AABBs penetrate (penetration > 0).
bool testAABBAABB(Vec2 posA, Vec2 halfExtentsA,
                  Vec2 posB, Vec2 halfExtentsB,
                  Contact& outContact,
                  BodyHandle bodyA = BodyHandle::null(),
                  BodyHandle bodyB = BodyHandle::null()) noexcept;

/// Tests collision between a circle and an AABB.
///
/// \param circlePos        Center of circle A.
/// \param radius           Radius of circle A (> 0).
/// \param aabbPos          Center of AABB B.
/// \param aabbHalfExtents  Half-width (x) and half-height (y) of box B (> 0).
/// \param outContact       Populated with normal (pointing from circle A to AABB B),
///                         penetration, and point if overlapping.
/// \param bodyA            Optional handle to assign to outContact.bodyA.
/// \param bodyB            Optional handle to assign to outContact.bodyB.
/// \return                 True if circle and AABB penetrate (penetration > 0).
bool testCircleAABB(Vec2 circlePos, float radius,
                    Vec2 aabbPos, Vec2 aabbHalfExtents,
                    Contact& outContact,
                    BodyHandle bodyA = BodyHandle::null(),
                    BodyHandle bodyB = BodyHandle::null()) noexcept;

// ── Broad-Phase Selector ─────────────────────────────────────────────────────

enum class BroadPhaseType : uint8_t {
    BruteForce          = 0, // Reference O(N^2) pairwise scanner (initial default)
    SpatialGrid         = 1, // Uniform spatial grid broad-phase acceleration (sequential)
    SpatialGridParallel = 2, // Parallel uniform spatial grid broad-phase pipeline
};

// ── Handle-based Dispatcher ──────────────────────────────────────────────────

/// Tests collision between two active bodies in the engine.
///
/// Validates handles. If handleA == handleB, or either handle is invalid,
/// returns false without modifying outContact.
bool testCollision(const Engine& engine,
                   BodyHandle handleA,
                   BodyHandle handleB,
                   Contact& outContact) noexcept;

/// Low-level dispatcher testing two bodies identified by their SoA slots and handles.
/// Returns true if narrowphase detects an actual collision with positive penetration.
bool testCollisionSlots(const Engine& engine,
                        uint32_t slotA,
                        uint32_t slotB,
                        Contact& outContact,
                        BodyHandle handleA,
                        BodyHandle handleB) noexcept;

// ── Reference Brute-force Pair Generation ────────────────────────────────────

/// Scans all unique unordered pairs (i, j) with 0 <= i < j < engine.bodyCount().
///
/// - Self-pairs (i == j) are never generated.
/// - Duplicate unordered pairs (j, i) are never generated.
/// - Static-static pairs are omitted.
/// - Reuses outContacts buffer (clears it first, does not deallocate capacity).
void detectCollisionsBruteForce(const Engine& engine, std::vector<Contact>& outContacts);

/// Convenience overload returning a newly allocated vector of detected contacts.
[[nodiscard]] std::vector<Contact> detectCollisionsBruteForce(const Engine& engine);

} // namespace PhysicsEngine
