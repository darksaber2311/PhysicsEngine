#pragma once

// Engine.h -- PhysicsEngine2D simulation engine.
//
// Architecture overview:
//   Engine owns all simulation state in Structure of Arrays (SoA) layout.
//   No rendering dependency — zero GLFW/OpenGL includes.
//
// Memory layout:
//   All SoA arrays are pre-allocated to WorldConfig::maxBodies at construction
//   and never reallocated. Steady-state simulation (update/add/remove) does not
//   trigger heap allocations beyond the free-list operations (stack push/pop).
//
// Handle lifetime contract:
//   - addBody() returns a BodyHandle. The handle is valid until removeBody()
//     is called with that handle, or clear() is called.
//   - removeBody() increments the generation for that handle slot, invalidating
//     previously issued handles for that slot.
//   - Swap-and-pop is used internally to keep SoA data contiguous. The SoA
//     slot of the moved body changes, but its handle remains valid because the
//     handle-table entry is updated.
//   - Accessing a body via an invalid or stale handle is always safe: all
//     getters return a documented default value, and all setters no-op.
//
// Thread safety:
//   The Engine is NOT thread-safe in Milestone 2. Multithreading is added in
//   Milestone 7 after the single-threaded baseline is correct and benchmarked.

#include "PhysicsEngine/Math/Vec2.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Physics/WorldConfig.h"

#include <cstdint>
#include <vector>

namespace PhysicsEngine {

class Engine {
public:
    // ── Construction / destruction ─────────────────────────────────────────

    /// Constructs the engine with the given configuration.
    /// Pre-allocates all SoA arrays and free lists to config.maxBodies.
    /// If config.maxBodies == 0, it is treated as 1 (prevents degenerate state).
    explicit Engine(WorldConfig config = {});
    ~Engine() = default;

    // Non-copyable: the engine owns large pre-allocated arrays.
    Engine(const Engine&)            = delete;
    Engine& operator=(const Engine&) = delete;

    // Movable.
    Engine(Engine&&)            = default;
    Engine& operator=(Engine&&) = default;

    // ── Body lifetime ──────────────────────────────────────────────────────

    /// Creates a new body with the given descriptor.
    /// Returns BodyHandle::null() if capacity is exhausted.
    /// Validates desc and clamps/corrects invalid fields.
    [[nodiscard]] BodyHandle addBody(const BodyDesc& desc);

    /// Removes the body identified by handle.
    /// Uses swap-and-pop internally; all other handles remain valid.
    /// No-op if handle is null or already invalid.
    void removeBody(BodyHandle handle);

    /// Returns true if handle is non-null AND refers to a currently live body.
    [[nodiscard]] bool isValidHandle(BodyHandle handle) const noexcept;

    // ── World management ───────────────────────────────────────────────────

    /// Removes all bodies and resets the engine to its initial empty state.
    /// All previously issued handles become invalid.
    void clear();

    /// Changes the gravitational acceleration applied to dynamic bodies.
    /// Takes effect on the very next integrate() call.
    /// If either component of gravity is non-finite (NaN or inf), the call
    /// is silently ignored and the existing gravity is preserved.
    void setGravity(Vec2 gravity) noexcept;

    // ── Queries ────────────────────────────────────────────────────────────

    /// Number of currently active bodies.
    [[nodiscard]] uint32_t bodyCount()    const noexcept;

    /// Maximum number of bodies this engine can hold (from WorldConfig).
    [[nodiscard]] uint32_t bodyCapacity() const noexcept;

    /// Returns the WorldConfig used to construct this engine.
    [[nodiscard]] const WorldConfig& config() const noexcept;

    // ── Position / velocity ────────────────────────────────────────────────

    /// Returns the world-space position of the body.
    /// Returns (0,0) if handle is invalid.
    [[nodiscard]] Vec2 getPosition(BodyHandle handle) const noexcept;

    /// Sets the world-space position.  No-op if handle is invalid.
    void setPosition(BodyHandle handle, Vec2 position);

    /// Returns the velocity (m/s).  Returns (0,0) if handle is invalid.
    [[nodiscard]] Vec2 getVelocity(BodyHandle handle) const noexcept;

    /// Sets the velocity.  No-op if handle is invalid.
    void setVelocity(BodyHandle handle, Vec2 velocity);

    // ── Mass / material ────────────────────────────────────────────────────

    /// Returns inverse mass (1/kg).  0 indicates a static body.
    /// Returns 0 if handle is invalid (safe default — static-body semantics).
    [[nodiscard]] float getInverseMass(BodyHandle handle) const noexcept;

    /// Returns true if this is a static body (inverseMass == 0).
    /// Returns true if handle is invalid (conservative safe default).
    [[nodiscard]] bool isStatic(BodyHandle handle) const noexcept;

    /// Returns the coefficient of restitution [0, 1].
    /// Returns 0 if handle is invalid.
    [[nodiscard]] float getRestitution(BodyHandle handle) const noexcept;

    /// Sets the coefficient of restitution [0, 1].
    /// Clamped to [0, 1]. No-op if handle is invalid or restitution is non-finite.
    void setRestitution(BodyHandle handle, float restitution);

    // ── Shape ──────────────────────────────────────────────────────────────

    [[nodiscard]] ShapeType getShapeType(BodyHandle handle) const noexcept;
    [[nodiscard]] float     getRadius    (BodyHandle handle) const noexcept;
    [[nodiscard]] float     getHalfHeight(BodyHandle handle) const noexcept;

    // ── Force accumulation (used by Milestone 3 integration) ───────────────

    /// Adds a force to the accumulated force for one step.
    /// No-op for static bodies (inverseMass == 0) and invalid handles.
    void addForce(BodyHandle handle, Vec2 force);

    /// Resets accumulated forces to zero for all bodies.
    /// Called at the start of each simulation step.
    void clearForces();

    // ── Direct SoA read access for the simulation loop ─────────────────────
    // These return read-only spans over the contiguous SoA arrays.
    // Valid for indices [0, bodyCount()).  Used by the integration and
    // broad-phase code in later milestones to avoid per-body handle lookups.

    [[nodiscard]] const Vec2*      positionsData()      const noexcept;
    [[nodiscard]] const Vec2*      velocitiesData()     const noexcept;
    [[nodiscard]] const Vec2*      forcesData()         const noexcept;
    [[nodiscard]] const float*     inverseMassesData()  const noexcept;
    [[nodiscard]] const float*     restitutionsData()   const noexcept;
    [[nodiscard]] const ShapeType* shapeTypesData()     const noexcept;
    [[nodiscard]] const float*     radiiData()          const noexcept;
    [[nodiscard]] const float*     halfHeightsData()    const noexcept;

    /// Reconstructs the valid BodyHandle for the body occupying the given SoA slot.
    /// Returns BodyHandle::null() if slot >= bodyCount().
    [[nodiscard]] BodyHandle handleFromSlot(uint32_t slot) const noexcept;

    // Mutable SoA access — used by the integration step to write updated state.
    [[nodiscard]] Vec2*  positionsMut()   noexcept;
    [[nodiscard]] Vec2*  velocitiesMut()  noexcept;
    [[nodiscard]] Vec2*  forcesMut()      noexcept;

private:
    // ── Validation helpers ─────────────────────────────────────────────────

    /// Returns the SoA slot for a valid handle, or kInvalidSlot if invalid.
    [[nodiscard]] uint32_t soaSlotOf(BodyHandle handle) const noexcept;

    // ── Configuration ──────────────────────────────────────────────────────
    WorldConfig config_;

    // ── SoA arrays ────────────────────────────────────────────────────────
    // All vectors are reserved to config_.maxBodies at construction and
    // never reallocated. Active data lives at indices [0, activeCount_).
    // Inactive indices may contain stale data — never read them without
    // first verifying the corresponding handle is valid.
    std::vector<Vec2>      positions_;
    std::vector<Vec2>      velocities_;
    std::vector<Vec2>      forces_;
    std::vector<float>     inverseMasses_;
    std::vector<float>     restitutions_;
    std::vector<ShapeType> shapeTypes_;
    std::vector<float>     radii_;
    std::vector<float>     halfHeights_;

    // ── Handle table ───────────────────────────────────────────────────────
    // handleSlots_[i]       : the SoA index for handle-table slot i.
    //                         kInvalidSlot = no body is currently using slot i.
    // handleGenerations_[i] : incremented each time slot i is reused.
    //                         A handle is valid iff
    //                           handle.generation == handleGenerations_[handle.index]
    //                           && handleSlots_[handle.index] != kInvalidSlot
    static constexpr uint32_t kInvalidSlot = std::numeric_limits<uint32_t>::max();

    std::vector<uint32_t> handleSlots_;
    std::vector<uint32_t> handleGenerations_;

    // ── Reverse mapping: SoA slot -> handle-table index ────────────────────
    // reverseHandle_[soaSlot] = the handle-table index whose body is at soaSlot.
    // Needed during swap-and-pop to fix up the moved body's handle-table entry.
    std::vector<uint32_t> reverseHandle_;

    // ── Bookkeeping ────────────────────────────────────────────────────────
    uint32_t              activeCount_ = 0;
    // No SoA free list: new bodies always go at SoA index activeCount_.
    // After swap-and-pop removal, activeCount_ is decremented, making that
    // slot available for the next addBody call automatically.
    std::vector<uint32_t> handleFreeList_;  // free handle-table slots (LIFO)
};

} // namespace PhysicsEngine
