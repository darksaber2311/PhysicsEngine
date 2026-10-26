// Integration.cpp -- Semi-implicit (symplectic) Euler integration step.
//
// See Integration.h for full design rationale and API contract.
//
// Implementation notes:
//   - Iterates over the contiguous SoA range [0, bodyCount()).
//   - Uses raw pointer arithmetic (positionsMut(), velocitiesMut(),
//     inverseMassesData(), forcesData()) for cache-friendly sequential access.
//   - A single branch per body skips static bodies (inverseMass == 0).
//   - No heap allocation; no function calls in the hot path beyond this file.
//   - The loop is deliberately simple: correct single-threaded baseline first.
//     Parallelism is added in Milestone 7 after benchmarking this version.

#include "PhysicsEngine/Physics/Integration.h"

#include <cmath>    // std::isfinite

namespace PhysicsEngine {

void integrate(Engine& engine, float dt)
{
    // Validate timestep.  A non-positive or non-finite dt means the caller
    // has a logic error (negative time, NaN from upstream computation, etc.).
    // We treat this as a no-op to avoid corrupting simulation state.
    if (dt <= 0.f || !std::isfinite(dt)) {
        return;
    }

    const uint32_t n        = engine.bodyCount();
    const Vec2     gravity  = engine.config().gravity;

    // Raw SoA pointers.  Valid for indices [0, n).
    Vec2*        pos  = engine.positionsMut();
    Vec2*        vel  = engine.velocitiesMut();
    const Vec2*  f    = engine.forcesData();
    const float* invM = engine.inverseMassesData();

    for (uint32_t i = 0; i < n; ++i) {
        const float invMass = invM[i];

        // Skip static bodies entirely.  Their position and velocity never
        // change through integration (only direct setPosition/setVelocity calls
        // can move them, which is intentional).
        if (invMass == 0.f) continue;

        // ── Step 1: update velocity ──────────────────────────────────────────
        //
        // Total acceleration = gravity + external_force * inverseMass
        //
        // Gravity is a uniform acceleration; applying it directly avoids the
        // F = m*g -> a = F/m roundtrip (exact, one fewer multiply).
        //
        // External forces are already in world-space Newtons.  Multiply by
        // inverseMass to get acceleration.
        //
        //   v_new = v + (gravity + f * invM) * dt
        //         = v + gravity*dt + f*invM*dt

        vel[i] += gravity * dt;           // gravity term
        vel[i] += f[i] * (invMass * dt);  // external force term

        // ── Step 2: update position ──────────────────────────────────────────
        //
        // Use the NEW velocity (semi-implicit / symplectic Euler):
        //   x_new = x + v_new * dt
        //
        // This one-line difference from explicit Euler makes the integrator
        // symplectic, preventing energy drift in long simulations.

        pos[i] += vel[i] * dt;
    }

    // ── Step 3: clear consumed forces ────────────────────────────────────────
    //
    // External forces accumulated in forces_[] represent forces active during
    // this step. Now that their impulse (F * dt) has been transferred into
    // velocity, they are consumed. Clearing them immediately after integration
    // guarantees they are not accidentally re-applied on subsequent substeps or
    // frames, while ensuring any force applied immediately prior to integrate()
    // is fully consumed.
    engine.clearForces();
}

} // namespace PhysicsEngine
