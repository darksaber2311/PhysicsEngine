#pragma once

// Body.h -- BodyHandle, ShapeType, and BodyDesc for PhysicsEngine2D.
//
// BodyHandle design:
//   A handle is a (index, generation) pair.
//   - index      : indexes into the Engine's handle-table (NOT the SoA arrays directly).
//   - generation : must match Engine::handleGenerations_[index] to be valid.
//
//   The handle-table entry handleSlots_[index] stores the actual SoA slot.
//   On swap-and-pop removal, the SoA slot of the moved body changes, but the
//   handle-table entry is updated, so existing handles remain valid.
//   On body removal, handleGenerations_[index] is incremented, invalidating
//   any previously issued handles that referenced that handle-table slot.
//
//   Consequence:
//     - A handle to the REMOVED body becomes invalid (generation mismatch).
//     - Handles to all OTHER bodies remain valid regardless of swap-and-pop.
//     - Reusing a slot after removal: the new body gets the next generation,
//       so the old handle cannot access the new body.

#include "PhysicsEngine/Math/Vec2.h"

#include <cstdint>
#include <limits>

namespace PhysicsEngine {

// ── BodyHandle ────────────────────────────────────────────────────────────────

struct BodyHandle {
    static constexpr uint32_t kNullIndex = std::numeric_limits<uint32_t>::max();

    uint32_t index      = kNullIndex;
    uint32_t generation = 0;

    /// Returns true if this handle was explicitly set to null (not created by
    /// addBody). Note: isNull() does NOT check whether the body is still alive;
    /// use Engine::isValidHandle() for that.
    [[nodiscard]] constexpr bool isNull() const noexcept {
        return index == kNullIndex;
    }

    /// Returns the canonical null handle.
    [[nodiscard]] static constexpr BodyHandle null() noexcept {
        return {};   // Default: index = kNullIndex, generation = 0
    }

    // Two handles are equal if both fields match.
    [[nodiscard]] constexpr bool operator==(BodyHandle rhs) const noexcept {
        return index == rhs.index && generation == rhs.generation;
    }
    [[nodiscard]] constexpr bool operator!=(BodyHandle rhs) const noexcept {
        return !(*this == rhs);
    }
};

// ── ShapeType ─────────────────────────────────────────────────────────────────

enum class ShapeType : uint8_t {
    Circle = 0,  // Defined by a radius (halfHeight unused)
    AABB   = 1,  // Defined by half-extents (halfWidth, halfHeight)
};

// ── BodyDesc ──────────────────────────────────────────────────────────────────

/// Descriptor passed to Engine::addBody().
/// Collects all per-body properties so addBody() has a single, clear input.
///
/// Validation rules (enforced by addBody):
///   - mass:       must be >= 0. 0 = static body (infinite mass).
///                 Negative mass is invalid and treated as 0 (static).
///   - restitution: clamped to [0, 1].
///   - radius:     must be > 0 for Circle; must be > 0 for AABB half-extents.
///                 Non-positive values produce a 1.0 unit default.
struct BodyDesc {
    Vec2      position    = {0.f, 0.f};
    Vec2      velocity    = {0.f, 0.f};
    float     mass        = 1.f;        // kg; 0 = static
    float     restitution = 0.5f;       // coefficient of restitution [0, 1]
    ShapeType shapeType   = ShapeType::Circle;
    float     radius      = 1.f;        // circle radius OR AABB half-width
    float     halfHeight  = 0.f;        // AABB half-height; unused for circles
};

} // namespace PhysicsEngine
