#pragma once

// WorldConfig.h -- Immutable configuration passed to Engine at construction.
//
// All capacity and tuning parameters live here so they are visible at the
// call site and can be documented centrally.

#include "PhysicsEngine/Math/Vec2.h"

#include <cstdint>

namespace PhysicsEngine {

/// Configuration passed to Engine at construction time.
/// All fields are read by the Engine during its constructor; they are not
/// observed after that point (the Engine copies or acts on them once).
struct WorldConfig {
    /// Maximum number of simultaneously active bodies.
    /// All SoA arrays are preallocated to this capacity and never grown.
    /// addBody() returns BodyHandle::null() when this limit is reached.
    /// Must be >= 1.
    uint32_t maxBodies = 1024;

    /// Gravitational acceleration applied to all dynamic bodies each step.
    /// Units: m/s^2. Negative y = downward.
    /// The integration loop (Milestone 3) reads this per step.
    Vec2 gravity = {0.f, -9.81f};

    /// Fixed simulation timestep in seconds.
    /// Not used until Milestone 3 (integration); stored here for later use.
    float fixedDt = 1.f / 120.f;
};

} // namespace PhysicsEngine
