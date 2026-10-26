#pragma once

// SimulationController.h -- Fixed-timestep simulation controller.
//
// Role and Objective:
//   Decouples the physics simulation rate from the rendering / frame rate.
//   Rendering may run at 60Hz, 144Hz, or variable display refresh rates, while
//   physics simulation demands a fixed, deterministic timestep (fixedDt) for
//   numerical stability, energy conservation, and repeatability.
//
// Key Responsibilities:
//   1. Frame Time Accumulation:
//      Accepts variable elapsed frame time (e.g., from glfwGetTime delta) and
//      accumulates it in an internal accumulator.
//   2. Fixed Physics Steps (advance):
//      Consumes fixedDt chunks from the accumulator, invoking the physics
//      integrator exactly once per chunk.
//   3. Spiral-of-Death Prevention (Catch-up Limiting):
//      Caps the maximum number of physics steps executed in a single frame to
//      maxSubsteps. If a slow frame causes the accumulator to exceed this limit,
//      the excess accumulated time is discarded (clamped to 0.f). This prevents
//      the cascade where heavy physics causes lower frame rates, demanding even
//      more physics steps per frame.
//   4. Interactive Control:
//      Supports pause, resume, accumulator reset, and explicit single-step
//      execution (e.g. stepping one frame while paused for debugging).
//   5. Completely Headless:
//      Zero OpenGL / GLFW dependencies. Usable in headless servers, tests,
//      benchmarks, and windowed GUI apps alike.
//   6. Zero Allocations:
//      Contains only primitive scalar state. No heap allocations on any path.
//
// Distinction between step() and advance():
//   - step(engine):
//       Directly executes exactly ONE fixed physics step of duration fixedDt,
//       advancing the simulation by fixedDt and clearing consumed forces.
//       Bypasses the accumulator and operates even when the controller is paused.
//       Used for explicit single-step debugging.
//   - advance(engine, frameTime):
//       Consumes elapsed real-world frame time, updates the accumulator, and
//       executes zero, one, or multiple fixed steps (up to maxSubsteps).
//       Does not advance physics when paused. Returns the number of steps taken.

#include "PhysicsEngine/Physics/Collision.h"
#include "PhysicsEngine/Physics/CollisionSolver.h"
#include "PhysicsEngine/Physics/Contact.h"
#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/SpatialGrid.h"

#include <cstdint>
#include <vector>

namespace PhysicsEngine {

class WorkerPool;

class SimulationController {
public:
    /// Default maximum catch-up steps allowed in a single advance() call.
    static constexpr uint32_t kDefaultMaxSubsteps = 8;

    /// Constructs a controller with a configurable catch-up substep limit.
    /// If maxSubsteps == 0, it is clamped to 1.
    explicit SimulationController(uint32_t maxSubsteps = kDefaultMaxSubsteps) noexcept;

    // ── Simulation stepping ──────────────────────────────────────────────────

    /// Advances the simulation by consuming elapsed real-world frame time.
    ///
    /// Algorithm:
    ///   1. Validates frameTime: if frameTime <= 0 or non-finite, returns 0.
    ///   2. If paused, returns 0 without modifying the accumulator.
    ///   3. Adds frameTime to the internal accumulator.
    ///   4. While accumulator >= fixedDt and steps < maxSubsteps:
    ///        step(engine);
    ///        accumulator -= fixedDt;
    ///        ++steps;
    ///   5. Spiral-of-death clamp: if accumulator >= fixedDt after maxSubsteps
    ///      have been executed, the excess accumulated time is discarded
    ///      (accumulator reset to 0.f) to prevent latency runaway.
    ///
    /// \param engine     The physics engine to advance.
    /// \param frameTime  Elapsed time since the previous frame in seconds.
    /// \return           Number of fixed physics steps executed (0 or more).
    uint32_t advance(Engine& engine, float frameTime);

    /// Executes exactly ONE fixed physics step (fixedDt = engine.config().fixedDt).
    ///
    /// Does NOT touch or consume the time accumulator.
    /// Operates even when the controller is paused (ideal for stepping while paused).
    /// Calls integrate(engine, fixedDt) and clears consumed forces.
    void step(Engine& engine);

    /// Explicit alias for step() to clarify single-step debugging intent.
    void singleStep(Engine& engine) { step(engine); }

    // ── State control ────────────────────────────────────────────────────────

    /// Pauses the automatic progression of physics in advance().
    void pause() noexcept { paused_ = true; }

    /// Resumes automatic progression of physics in advance().
    void resume() noexcept { paused_ = false; }

    /// Sets the paused state directly.
    void setPaused(bool paused) noexcept { paused_ = paused; }

    /// Returns true if the controller is currently paused.
    [[nodiscard]] bool isPaused() const noexcept { return paused_; }

    /// Resets the time accumulator to 0.0. Useful when resuming after an
    /// interruption (e.g. modal dialog, window resizing).
    void resetAccumulator() noexcept { accumulator_ = 0.f; }

    /// Returns the currently accumulated unsimulated time in seconds [0, fixedDt).
    [[nodiscard]] float accumulator() const noexcept { return accumulator_; }

    /// Returns the linear interpolation alpha for rendering state blending:
    ///   alpha = accumulator / fixedDt in [0.0, 1.0)
    /// Returns 0.0 if fixedDt <= 0.
    [[nodiscard]] float subStepAlpha(const Engine& engine) const noexcept;

    // ── Configuration ────────────────────────────────────────────────────────

    /// Maximum number of substeps allowed per advance() call.
    [[nodiscard]] uint32_t maxSubsteps() const noexcept { return maxSubsteps_; }

    /// Sets the maximum substep limit. Clamped to at least 1.
    void setMaxSubsteps(uint32_t maxSubsteps) noexcept {
        maxSubsteps_ = (maxSubsteps > 0) ? maxSubsteps : 1;
    }

    /// Returns the active broad-phase collision detection algorithm.
    [[nodiscard]] BroadPhaseType broadPhaseType() const noexcept { return broadPhaseType_; }

    /// Sets the broad-phase collision detection algorithm.
    void setBroadPhaseType(BroadPhaseType type) noexcept { broadPhaseType_ = type; }

    /// Returns the spatial grid broad-phase instance.
    [[nodiscard]] SpatialGrid& spatialGrid() noexcept { return spatialGrid_; }
    [[nodiscard]] const SpatialGrid& spatialGrid() const noexcept { return spatialGrid_; }

    /// Returns the collision solver configuration.
    [[nodiscard]] const SolverConfig& solverConfig() const noexcept { return solverConfig_; }

    /// Sets the collision solver configuration.
    void setSolverConfig(const SolverConfig& config) noexcept { solverConfig_ = config; }

    /// Returns the attached worker pool, if any.
    [[nodiscard]] WorkerPool* workerPool() const noexcept { return workerPool_; }

    /// Attaches a persistent worker pool for parallel execution.
    void setWorkerPool(WorkerPool* pool) noexcept { workerPool_ = pool; }

    /// Returns the list of contacts detected and resolved during the most recent step.
    [[nodiscard]] const std::vector<Contact>& lastContacts() const noexcept { return contactsBuffer_; }

    /// Clears the contact buffer (e.g. on scene reset).
    void clearContacts() noexcept { contactsBuffer_.clear(); }

private:
    uint32_t             maxSubsteps_;
    float                accumulator_    = 0.f;
    bool                 paused_         = false;
    BroadPhaseType       broadPhaseType_ = BroadPhaseType::BruteForce;
    SpatialGrid          spatialGrid_;
    SolverConfig         solverConfig_{};
    std::vector<Contact> contactsBuffer_;
    WorkerPool*          workerPool_     = nullptr;
};

} // namespace PhysicsEngine
