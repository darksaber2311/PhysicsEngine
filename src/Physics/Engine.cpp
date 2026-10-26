// Engine.cpp -- PhysicsEngine2D simulation engine implementation.
//
// See Engine.h for the full design rationale and invariants.
//
// SoA slot allocation strategy (simplified after design review):
//   - Active bodies always occupy SoA indices [0, activeCount_).
//   - addBody() always places the new body at SoA index activeCount_.
//   - removeBody() uses swap-and-pop: moves the body at lastSoaSlot into
//     the removed slot, then decrements activeCount_.
//   - No SoA free list is needed: the next available SoA slot is always
//     activeCount_ (since active data is kept contiguous).
//
// Handle-table allocation strategy:
//   - Handle-table indices are independent of SoA slots.
//   - A handleFreeList_ tracks available handle-table indices.
//   - On addBody: pop from handleFreeList_, wire up handleSlots_ and reverseHandle_.
//   - On removeBody: increment generation, push handle-table index back.
//
// Key invariants:
//   1. Active bodies occupy SoA[0 .. activeCount_).
//   2. handleSlots_[h.index] is the SoA index for valid handle h.
//   3. reverseHandle_[soaSlot] is the handle-table index for the body at soaSlot.
//   4. handleGenerations_[h.index] == h.generation for any valid handle h.
//   5. No SoA vector is ever reallocated after construction.

#include "PhysicsEngine/Physics/Engine.h"

#include <cassert>
#include <cmath>

namespace PhysicsEngine {

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers (file-local)
// ─────────────────────────────────────────────────────────────────────────────

namespace {

/// Convert a mass value to its inverse.
/// mass <= 0 is treated as a static body (infinite mass) -> inverseMass = 0.
/// Non-finite or negative mass is invalid physically; we silently make it static.
float toInverseMass(float mass) noexcept
{
    if (!std::isfinite(mass) || mass <= 0.f) return 0.f;
    const float invM = 1.f / mass;
    return (std::isfinite(invM) && invM > 0.f) ? invM : 0.f;
}

/// Clamp a restitution coefficient to [0, 1].
float clampRestitution(float r) noexcept
{
    if (!std::isfinite(r) || r < 0.f) return 0.f;
    if (r > 1.f) return 1.f;
    return r;
}

/// Sanitise a shape dimension (radius or half-extent).
/// Must be strictly positive and finite. If not, return 1.f as a safe fallback.
float sanitiseDimension(float d) noexcept
{
    return (std::isfinite(d) && d > 0.f) ? d : 1.f;
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

Engine::Engine(WorldConfig config)
    : config_(config)
{
    if (config_.maxBodies == 0) {
        config_.maxBodies = 1;  // prevent degenerate state
    }
    const uint32_t cap = config_.maxBodies;

    // ── Pre-allocate SoA arrays ─────────────────────────────────────────────
    // resize() to cap so element access via operator[] is always valid.
    // We write slots by direct index, never via push_back, to keep the
    // active region [0, activeCount_) contiguous and allocation-free.
    positions_    .resize(cap);
    velocities_   .resize(cap);
    forces_       .resize(cap);
    inverseMasses_.resize(cap, 0.f);
    restitutions_ .resize(cap, 0.f);
    shapeTypes_   .resize(cap, ShapeType::Circle);
    radii_        .resize(cap, 1.f);
    halfHeights_  .resize(cap, 0.f);

    // ── Pre-allocate handle table and reverse map ───────────────────────────
    handleSlots_      .resize(cap, kInvalidSlot);
    handleGenerations_.resize(cap, 0);
    reverseHandle_    .resize(cap, kInvalidSlot);

    // ── Populate handle free list ───────────────────────────────────────────
    // Push in reverse order so that handle index 0 is allocated first (top of
    // the stack = back of the vector = what pop_back() returns).
    // This gives deterministic, ascending handle indices in tests.
    handleFreeList_.reserve(cap);
    for (uint32_t i = cap; i-- > 0;) {
        handleFreeList_.push_back(i);
    }
    // activeCount_ is initialised to 0 (first addBody places data at index 0).
}

// ─────────────────────────────────────────────────────────────────────────────
// Body lifetime
// ─────────────────────────────────────────────────────────────────────────────

BodyHandle Engine::addBody(const BodyDesc& desc)
{
    // Reject non-finite position or velocity to prevent NaN/inf propagation
    if (!std::isfinite(desc.position.x) || !std::isfinite(desc.position.y) ||
        !std::isfinite(desc.velocity.x) || !std::isfinite(desc.velocity.y)) {
        return BodyHandle::null();
    }

    // Capacity check.
    // activeCount_ >= maxBodies: SoA is full.
    // handleFreeList_ empty: no handle slots available (should match SoA full).
    if (activeCount_ >= config_.maxBodies || handleFreeList_.empty()) {
        return BodyHandle::null();
    }

    // ── Claim a handle-table slot ───────────────────────────────────────────
    const uint32_t handleIdx = handleFreeList_.back();
    handleFreeList_.pop_back();

    // ── New body always goes at the end of the active SoA range ────────────
    const uint32_t soaSlot = activeCount_;

    // ── Write SoA data ──────────────────────────────────────────────────────
    positions_    [soaSlot] = desc.position;
    velocities_   [soaSlot] = desc.velocity;
    forces_       [soaSlot] = {0.f, 0.f};
    inverseMasses_[soaSlot] = toInverseMass(desc.mass);
    restitutions_ [soaSlot] = clampRestitution(desc.restitution);
    shapeTypes_   [soaSlot] = desc.shapeType;
    radii_        [soaSlot] = sanitiseDimension(desc.radius);
    halfHeights_  [soaSlot] = (desc.shapeType == ShapeType::AABB)
                                  ? sanitiseDimension(desc.halfHeight)
                                  : 0.f;

    // ── Wire up handle table and reverse map ────────────────────────────────
    handleSlots_[handleIdx]     = soaSlot;
    reverseHandle_[soaSlot]     = handleIdx;

    ++activeCount_;

    // The generation is whatever is currently in handleGenerations_[handleIdx].
    // It was incremented when the previous owner of this handle slot was removed
    // (or is 0 if this is the first occupant).
    return BodyHandle{handleIdx, handleGenerations_[handleIdx]};
}

void Engine::removeBody(BodyHandle handle)
{
    // Null and invalid handles are silently ignored.
    if (handle.isNull() || !isValidHandle(handle)) return;

    const uint32_t soaSlot     = handleSlots_[handle.index];
    const uint32_t lastSoaSlot = activeCount_ - 1;

    // ── Swap-and-pop ────────────────────────────────────────────────────────
    // Move the last active body into the vacated slot, then shrink the
    // active range by one. This keeps [0, activeCount_) contiguous.
    if (soaSlot != lastSoaSlot) {
        // Copy every SoA array entry from lastSoaSlot to soaSlot.
        positions_    [soaSlot] = positions_    [lastSoaSlot];
        velocities_   [soaSlot] = velocities_   [lastSoaSlot];
        forces_       [soaSlot] = forces_       [lastSoaSlot];
        inverseMasses_[soaSlot] = inverseMasses_[lastSoaSlot];
        restitutions_ [soaSlot] = restitutions_ [lastSoaSlot];
        shapeTypes_   [soaSlot] = shapeTypes_   [lastSoaSlot];
        radii_        [soaSlot] = radii_        [lastSoaSlot];
        halfHeights_  [soaSlot] = halfHeights_  [lastSoaSlot];

        // Fix up the moved body's handle-table entry so its handle still works.
        const uint32_t movedHandleIdx = reverseHandle_[lastSoaSlot];
        handleSlots_  [movedHandleIdx] = soaSlot;   // now lives at soaSlot
        reverseHandle_[soaSlot]        = movedHandleIdx;
    }

    // ── Invalidate the removed handle ───────────────────────────────────────
    // Increment generation: any handle carrying the old generation is now stale.
    ++handleGenerations_[handle.index];
    handleSlots_[handle.index] = kInvalidSlot;

    // Return the handle-table slot for reuse.
    handleFreeList_.push_back(handle.index);

    --activeCount_;
}

bool Engine::isValidHandle(BodyHandle handle) const noexcept
{
    if (handle.isNull())                                           return false;
    if (handle.index >= config_.maxBodies)                         return false;
    if (handle.generation != handleGenerations_[handle.index])     return false;
    if (handleSlots_[handle.index] == kInvalidSlot)                return false;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// World management
// ─────────────────────────────────────────────────────────────────────────────

void Engine::clear()
{
    const uint32_t cap = config_.maxBodies;

    // Invalidate all currently live handles by incrementing their generations.
    for (uint32_t i = 0; i < cap; ++i) {
        if (handleSlots_[i] != kInvalidSlot) {
            ++handleGenerations_[i];
            handleSlots_[i] = kInvalidSlot;
        }
        reverseHandle_[i] = kInvalidSlot;
    }

    // Rebuild the handle free list (all slots available again).
    handleFreeList_.clear();
    for (uint32_t i = cap; i-- > 0;) {
        handleFreeList_.push_back(i);
    }

    activeCount_ = 0;
    // No need to touch soaFreeList_ — it is never used for allocation.
}

// ─────────────────────────────────────────────────────────────────────────────
// Queries
// ─────────────────────────────────────────────────────────────────────────────

uint32_t Engine::bodyCount()    const noexcept { return activeCount_; }
uint32_t Engine::bodyCapacity() const noexcept { return config_.maxBodies; }
const WorldConfig& Engine::config() const noexcept { return config_; }

void Engine::setGravity(Vec2 gravity) noexcept
{
    // Reject non-finite gravity vectors — NaN or inf gravity would corrupt
    // every dynamic body's velocity on the next integrate() call.
    if (!std::isfinite(gravity.x) || !std::isfinite(gravity.y)) return;
    config_.gravity = gravity;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private: soaSlotOf
// ─────────────────────────────────────────────────────────────────────────────

uint32_t Engine::soaSlotOf(BodyHandle handle) const noexcept
{
    if (!isValidHandle(handle)) return kInvalidSlot;
    return handleSlots_[handle.index];
}

// ─────────────────────────────────────────────────────────────────────────────
// Position / velocity
// ─────────────────────────────────────────────────────────────────────────────

Vec2 Engine::getPosition(BodyHandle handle) const noexcept
{
    const uint32_t slot = soaSlotOf(handle);
    return (slot != kInvalidSlot) ? positions_[slot] : Vec2{};
}

void Engine::setPosition(BodyHandle handle, Vec2 position)
{
    // Reject non-finite inputs to prevent NaN/inf from entering the SoA arrays
    // and propagating through the integration loop.
    if (!std::isfinite(position.x) || !std::isfinite(position.y)) return;
    const uint32_t slot = soaSlotOf(handle);
    if (slot != kInvalidSlot) positions_[slot] = position;
}

Vec2 Engine::getVelocity(BodyHandle handle) const noexcept
{
    const uint32_t slot = soaSlotOf(handle);
    return (slot != kInvalidSlot) ? velocities_[slot] : Vec2{};
}

void Engine::setVelocity(BodyHandle handle, Vec2 velocity)
{
    // Reject non-finite inputs — same convention as setPosition.
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y)) return;
    const uint32_t slot = soaSlotOf(handle);
    if (slot != kInvalidSlot) velocities_[slot] = velocity;
}

// ─────────────────────────────────────────────────────────────────────────────
// Mass / material
// ─────────────────────────────────────────────────────────────────────────────

float Engine::getInverseMass(BodyHandle handle) const noexcept
{
    const uint32_t slot = soaSlotOf(handle);
    return (slot != kInvalidSlot) ? inverseMasses_[slot] : 0.f;
}

bool Engine::isStatic(BodyHandle handle) const noexcept
{
    return getInverseMass(handle) == 0.f;
}

float Engine::getRestitution(BodyHandle handle) const noexcept
{
    const uint32_t slot = soaSlotOf(handle);
    return (slot != kInvalidSlot) ? restitutions_[slot] : 0.f;
}

void Engine::setRestitution(BodyHandle handle, float restitution)
{
    if (!std::isfinite(restitution)) return;
    const uint32_t slot = soaSlotOf(handle);
    if (slot != kInvalidSlot) {
        restitutions_[slot] = clampRestitution(restitution);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Shape
// ─────────────────────────────────────────────────────────────────────────────

ShapeType Engine::getShapeType(BodyHandle handle) const noexcept
{
    const uint32_t slot = soaSlotOf(handle);
    return (slot != kInvalidSlot) ? shapeTypes_[slot] : ShapeType::Circle;
}

float Engine::getRadius(BodyHandle handle) const noexcept
{
    const uint32_t slot = soaSlotOf(handle);
    return (slot != kInvalidSlot) ? radii_[slot] : 0.f;
}

float Engine::getHalfHeight(BodyHandle handle) const noexcept
{
    const uint32_t slot = soaSlotOf(handle);
    return (slot != kInvalidSlot) ? halfHeights_[slot] : 0.f;
}

// ─────────────────────────────────────────────────────────────────────────────
// Force accumulation
// ─────────────────────────────────────────────────────────────────────────────

void Engine::addForce(BodyHandle handle, Vec2 force)
{
    const uint32_t slot = soaSlotOf(handle);
    if (slot == kInvalidSlot) return;
    if (inverseMasses_[slot] == 0.f) return;  // no-op for static bodies
    forces_[slot] += force;
}

void Engine::clearForces()
{
    for (uint32_t i = 0; i < activeCount_; ++i) {
        forces_[i] = {0.f, 0.f};
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Raw SoA data accessors
// ─────────────────────────────────────────────────────────────────────────────

const Vec2*      Engine::positionsData()     const noexcept { return positions_    .data(); }
const Vec2*      Engine::velocitiesData()    const noexcept { return velocities_   .data(); }
const Vec2*      Engine::forcesData()        const noexcept { return forces_       .data(); }
const float*     Engine::inverseMassesData() const noexcept { return inverseMasses_.data(); }
const float*     Engine::restitutionsData()  const noexcept { return restitutions_ .data(); }
const ShapeType* Engine::shapeTypesData()    const noexcept { return shapeTypes_   .data(); }
const float*     Engine::radiiData()         const noexcept { return radii_        .data(); }
const float*     Engine::halfHeightsData()   const noexcept { return halfHeights_  .data(); }

BodyHandle Engine::handleFromSlot(uint32_t slot) const noexcept
{
    if (slot >= activeCount_) {
        return BodyHandle::null();
    }
    const uint32_t handleIdx = reverseHandle_[slot];
    if (handleIdx == kInvalidSlot || handleIdx >= handleGenerations_.size()) {
        return BodyHandle::null();
    }
    return BodyHandle{handleIdx, handleGenerations_[handleIdx]};
}

Vec2*  Engine::positionsMut()  noexcept { return positions_ .data(); }
Vec2*  Engine::velocitiesMut() noexcept { return velocities_.data(); }
Vec2*  Engine::forcesMut()     noexcept { return forces_    .data(); }

} // namespace PhysicsEngine
