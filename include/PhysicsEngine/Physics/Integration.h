#pragma once

// Integration.h -- Semi-implicit (symplectic) Euler integration step.
//
// Design: Integration is a free function, not a method on Engine.
//   - Engine owns state (positions, velocities, forces, inverse masses).
//   - integrate() is a stateless algorithm that reads and writes Engine state
//     through Engine's existing public mutable accessors.
//   - This separation keeps Engine focused on data management and makes
//     the integration algorithm independently testable and replaceable.
//
// Integration method: semi-implicit (symplectic) Euler.
//   v(t+dt) = v(t) + a(t) * dt          -- velocity first
//   x(t+dt) = x(t) + v(t+dt) * dt       -- position uses NEW velocity
//
//   Using the updated velocity for the position step makes this integrator
//   symplectic: it conserves a modified energy, preventing the slow drift
//   (energy gain) that plagues explicit Euler in long simulations.
//   Local truncation error is O(dt^2), same as explicit Euler.
//
// Gravity treatment:
//   Gravity is a uniform acceleration (a = g), not a force per se.
//   We apply it directly to velocity as:
//       v += gravity * dt
//   This avoids the roundtrip F = m*g -> a = F/m = g and is exact.
//   Gravity is applied only to dynamic bodies (inverseMass > 0).
//
// Accumulated forces and force lifecycle:
//   External forces accumulated in Engine::forces_[] via addForce() represent
//   forces active during the upcoming timestep.
//   During integration:
//       a_external = force * inverseMass
//   Immediately following the integration of all bodies, integrate() calls
//   engine.clearForces() to clear consumed forces.
//   This guarantees that:
//     1. Forces applied prior to a step are fully consumed.
//     2. Forces are never applied repeatedly across subsequent substeps or frames.
//     3. Callers do not need to manually call clearForces() in normal usage,
//        though clearForces() remains available for explicit resets.
//
// Timestep validation:
//   dt must be strictly positive and finite.
//   dt <= 0 or non-finite dt: integrate() is a no-op (no state is modified).
//   This prevents divide-by-zero and NaN propagation if the caller passes
//   a bad timestep.

#include "PhysicsEngine/Physics/Engine.h"

namespace PhysicsEngine {

/// Advance the simulation by one fixed timestep using semi-implicit Euler.
///
/// \param engine  The engine whose bodies will be integrated.
/// \param dt      Timestep in seconds.  Must be > 0 and finite; no-op otherwise.
///
/// Per-body behaviour:
///   - Static bodies (inverseMass == 0): completely skipped. Position and
///     velocity are never modified, regardless of accumulated forces.
///   - Dynamic bodies (inverseMass > 0):
///       1. Gravity acceleration is added to velocity:  v += gravity * dt
///       2. Force-derived acceleration is added:        v += forces[i] * inverseMass[i] * dt
///       3. Position is updated with the NEW velocity:  x += v * dt
///
/// Preconditions (caller's responsibility):
///   - Accumulated forces set via Engine::addForce() reflect all external
///     forces for this step.
///   - Engine::clearForces() has been called at the start of this step
///     (before adding per-step forces) to avoid stale force carry-over.
void integrate(Engine& engine, float dt);

} // namespace PhysicsEngine
