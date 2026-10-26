#pragma once

// CollisionSolver.h -- Impulse-based collision response and contact resolution.
//
// Key Responsibilities:
//   1. Normal Impulse Resolution:
//      Solves relative normal velocities at contacts and applies instantaneous
//      momentum changes (impulses) to velocities.
//   2. Positional Correction (Linear Projection):
//      Resolves geometric interpenetration without injecting artificial kinetic energy,
//      distributing displacement according to inverse mass and respecting a penetration slop.
//   3. Material Properties:
//      Combines restitution coefficients via min(eA, eB), clamped to [0, 1].
//   4. Decoupling & Safety:
//      Completely headless, no GLFW/OpenGL dependencies.
//      Validates handles, guards against division by zero (e.g., static-static pairs),
//      and rejects non-finite numerical values.

#include "PhysicsEngine/Math/Vec2.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Physics/Contact.h"
#include "PhysicsEngine/Physics/Engine.h"

#include <span>
#include <vector>

namespace PhysicsEngine {

struct SolverConfig {
    /// Penetration allowance before positional correction is applied (meters).
    /// Penetrations less than or equal to slop are ignored to prevent jitter.
    float slop = 0.01f;

    /// Fraction of penetration excess to resolve in a single step [0, 1].
    /// A value of 0.4 resolves 40% of the penetration per step for smooth recovery.
    float percent = 0.4f;

    /// Maximum positional correction displacement magnitude per contact per step (meters).
    /// Clamps large corrections to avoid sudden object teleportation / tunneling.
    float maxCorrection = 0.5f;
};

/// Resolves normal velocity impulses for the given contacts.
///
/// Modifies only velocities in the engine; does not modify body positions.
/// Skips contacts with separating or stationary relative normal velocity (v_n >= 0).
void resolveVelocityImpulses(Engine& engine, std::span<const Contact> contacts);

/// Applies positional correction (linear projection) for the given contacts.
///
/// Modifies only positions in the engine; does not modify body velocities.
/// Displaces bodies along contact normals proportional to inverse mass.
void resolvePositionalCorrections(Engine& engine, std::span<const Contact> contacts,
                                 const SolverConfig& config = {});

/// Full collision resolution pass: resolves velocity impulses followed by positional correction.
void solveCollisions(Engine& engine, std::span<const Contact> contacts,
                     const SolverConfig& config = {});

// Convenience overloads for std::vector<Contact>
inline void resolveVelocityImpulses(Engine& engine, const std::vector<Contact>& contacts)
{
    resolveVelocityImpulses(engine, std::span<const Contact>(contacts.data(), contacts.size()));
}

inline void resolvePositionalCorrections(Engine& engine, const std::vector<Contact>& contacts,
                                        const SolverConfig& config = {})
{
    resolvePositionalCorrections(engine, std::span<const Contact>(contacts.data(), contacts.size()), config);
}

inline void solveCollisions(Engine& engine, const std::vector<Contact>& contacts,
                            const SolverConfig& config = {})
{
    solveCollisions(engine, std::span<const Contact>(contacts.data(), contacts.size()), config);
}

} // namespace PhysicsEngine
