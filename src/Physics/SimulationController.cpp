// SimulationController.cpp -- Fixed-timestep simulation controller implementation.
//
// See SimulationController.h for architectural rationale and API contract.

#include "PhysicsEngine/Physics/SimulationController.h"
#include "PhysicsEngine/Concurrency/WorkerPool.h"
#include "PhysicsEngine/Physics/Collision.h"
#include "PhysicsEngine/Physics/CollisionSolver.h"
#include "PhysicsEngine/Physics/Integration.h"

#include <algorithm>
#include <cmath>

namespace PhysicsEngine {

SimulationController::SimulationController(uint32_t maxSubsteps) noexcept
    : maxSubsteps_((maxSubsteps > 0) ? maxSubsteps : 1)
{
}

void SimulationController::step(Engine& engine)
{
    const float fixedDt = engine.config().fixedDt;
    if (fixedDt <= 0.f || !std::isfinite(fixedDt)) {
        return;
    }

    // 1. Numerical integration (semi-implicit Euler) & force clearing
    integrate(engine, fixedDt);

    // 2. Collision detection
    if (broadPhaseType_ == BroadPhaseType::SpatialGridParallel && workerPool_ != nullptr && workerPool_->workerCount() > 0) {
        spatialGrid_.detectCollisionsParallel(engine, contactsBuffer_, *workerPool_);
    } else if (broadPhaseType_ == BroadPhaseType::SpatialGrid || broadPhaseType_ == BroadPhaseType::SpatialGridParallel) {
        spatialGrid_.detectCollisions(engine, contactsBuffer_);
    } else {
        detectCollisionsBruteForce(engine, contactsBuffer_);
    }

    // 3. Collision resolution (velocity impulse + positional correction)
    solveCollisions(engine, contactsBuffer_, solverConfig_);
}

uint32_t SimulationController::advance(Engine& engine, float frameTime)
{
    // Reject invalid elapsed frame times (negative, zero, NaN, infinity).
    if (frameTime <= 0.f || !std::isfinite(frameTime)) {
        return 0;
    }

    // While paused, do not accumulate time and do not execute physics steps.
    if (paused_) {
        return 0;
    }

    const float fixedDt = engine.config().fixedDt;
    if (fixedDt <= 0.f || !std::isfinite(fixedDt)) {
        return 0;
    }

    accumulator_ += frameTime;

    uint32_t steps = 0;
    while (accumulator_ >= fixedDt && steps < maxSubsteps_) {
        step(engine);
        accumulator_ -= fixedDt;
        ++steps;
    }

    // Spiral-of-death prevention:
    // If the catch-up limit (maxSubsteps_) was reached but the accumulator
    // still contains remaining fixedDt chunks, the simulation cannot keep up
    // with real time. Discard the excess accumulated time to prevent latency
    // runaway and accumulator bloat.
    if (accumulator_ >= fixedDt) {
        accumulator_ = 0.f;
    }

    return steps;
}

float SimulationController::subStepAlpha(const Engine& engine) const noexcept
{
    const float fixedDt = engine.config().fixedDt;
    if (fixedDt <= 0.f || !std::isfinite(fixedDt)) {
        return 0.f;
    }

    const float alpha = accumulator_ / fixedDt;
    return std::clamp(alpha, 0.f, 1.f);
}

} // namespace PhysicsEngine
