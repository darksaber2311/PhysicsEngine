#pragma once

// SpatialGrid.h -- Uniform spatial grid broad-phase collision detection.
//
// Key Responsibilities:
//   1. Conservative AABB Calculation:
//      Computes an axis-aligned bounding box covering each active body (Circles and AABBs).
//   2. Uniform Spatial Hashing:
//      Maps world-space coordinates to integer grid coordinates using floor-based division.
//      Correctly handles negative coordinates and cell boundaries.
//   3. Duplicate Suppression:
//      Ensures each unordered candidate body pair (i, j) with i < j is tested at most once,
//      even when shapes span multiple grid cells.
//   4. Decoupling & Storage Reuse:
//      Completely headless, no GLFW/OpenGL dependencies.
//      Reuses internal cell-entry and candidate-pair buffers to prevent per-step dynamic allocations.
//   5. Narrowphase Dispatch:
//      Passes confirmed candidate pairs to testCollisionSlots() so emitted contacts conform
//      strictly to the established normal (A -> B) and positive penetration invariants.
//   6. Scoped Parallel Broad-Phase:
//      Supports parallelized broad-phase pipeline via detectCollisionsParallel() using a
//      persistent WorkerPool. Independent work units, zero shared vector mutations, deterministic
//      result merging, and configurable sequential threshold for small scenes.

#include "PhysicsEngine/Concurrency/WorkerPool.h"
#include "PhysicsEngine/Math/Vec2.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Physics/Collision.h"
#include "PhysicsEngine/Physics/Contact.h"
#include "PhysicsEngine/Physics/Engine.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace PhysicsEngine {

struct SpatialGridConfig {
    /// Size of each square grid cell along both X and Y axes (meters).
    /// Default: 2.0 meters. Must be > 0 and finite.
    float cellSize = 2.0f;

    /// Minimum body count required to dispatch work across worker threads in
    /// detectCollisionsParallel(). Scenes smaller than this threshold execute
    /// sequentially to avoid thread dispatch latency. Default: 256.
    uint32_t sequentialThreshold = 256;
};

class SpatialGrid {
public:
    static constexpr float    kDefaultCellSize            = 2.0f;
    static constexpr uint32_t kDefaultSequentialThreshold = 256;

    explicit SpatialGrid(float cellSize = kDefaultCellSize) noexcept;
    explicit SpatialGrid(const SpatialGridConfig& config) noexcept;

    /// Cell size getter and setter.
    /// Setting an invalid (<= 0 or non-finite) value falls back to kDefaultCellSize.
    [[nodiscard]] float cellSize() const noexcept { return cellSize_; }
    void setCellSize(float cellSize) noexcept;

    /// Sequential threshold getter and setter.
    [[nodiscard]] uint32_t sequentialThreshold() const noexcept { return sequentialThreshold_; }
    void setSequentialThreshold(uint32_t threshold) noexcept { sequentialThreshold_ = threshold; }

    /// Executes sequential broad-phase spatial culling and populates outContacts with confirmed
    /// narrow-phase contacts.
    ///
    /// - Static-static pairs are omitted.
    /// - Duplicate candidate pairs are eliminated.
    /// - Reuses internal buffers to avoid per-step heap allocations.
    /// - Clears outContacts first (preserves capacity).
    void detectCollisions(const Engine& engine, std::vector<Contact>& outContacts);

    /// Convenience overload returning newly allocated vector of contacts.
    [[nodiscard]] std::vector<Contact> detectCollisions(const Engine& engine);

    /// Executes parallelized broad-phase collision detection using a persistent WorkerPool.
    ///
    /// Pipeline:
    ///   - Falls back to sequential detectCollisions() if engine.bodyCount() < sequentialThreshold_
    ///     or pool.workerCount() == 0.
    ///   - Phase 1 (Parallel): Computes conservative AABBs and generates cell entries into
    ///     per-worker buffers (zero shared writes).
    ///   - Phase 2: Sorts cell entries by (cellKey, bodySlot).
    ///   - Phase 3A (Parallel): Generates candidate pairs from distinct cell slices without overlap.
    ///   - Phase 3B (Parallel): Tests oversized bodies against active bodies into local buffers.
    ///   - Phase 4: Merges, sorts, and deduplicates candidate pairs (100% deterministic canonical ordering).
    ///   - Phase 5 (Parallel): Dispatches narrow-phase tests across chunks into per-chunk contact buffers,
    ///     merging in deterministic chunk order to guarantee bit-for-bit repeatability.
    void detectCollisionsParallel(const Engine& engine,
                                  std::vector<Contact>& outContacts,
                                  WorkerPool& pool);

    /// Convenience overload returning newly allocated vector of contacts.
    [[nodiscard]] std::vector<Contact> detectCollisionsParallel(const Engine& engine,
                                                                WorkerPool& pool);

    /// Diagnostic query: returns the number of unique candidate pairs tested
    /// during the most recent detectCollisions or detectCollisionsParallel call.
    [[nodiscard]] size_t lastCandidatePairCount() const noexcept { return lastCandidatePairCount_; }

    /// Clears internal grid buffers and resets state.
    void clear() noexcept;

private:
    struct CellEntry {
        uint64_t cellKey;
        uint32_t bodySlot;

        bool operator<(const CellEntry& other) const noexcept {
            if (cellKey != other.cellKey) return cellKey < other.cellKey;
            return bodySlot < other.bodySlot;
        }
    };

    struct WorkerBuffer {
        std::vector<CellEntry> entries;
        std::vector<uint32_t>  oversized;
        std::vector<uint64_t>  candidatePairs;
    };

    static constexpr int32_t kMaxCellsPerBody = 128;
    static constexpr float   kMaxCoord        = 1e7f;

    float                             cellSize_            = kDefaultCellSize;
    uint32_t                          sequentialThreshold_ = kDefaultSequentialThreshold;
    size_t                            lastCandidatePairCount_ = 0;
    std::vector<CellEntry>            entries_;
    std::vector<uint64_t>             candidatePairs_;
    std::vector<uint32_t>             oversizedBodies_;

    // Per-worker and per-chunk scratch buffers for parallel execution
    std::vector<WorkerBuffer>         workerBuffers_;
    std::vector<std::vector<Contact>> chunkContacts_;
};

/// Standalone convenience wrapper for spatial grid collision detection (sequential).
void detectCollisionsSpatialGrid(const Engine& engine,
                                std::vector<Contact>& outContacts,
                                float cellSize = SpatialGrid::kDefaultCellSize);

/// Standalone convenience wrapper returning a vector of contacts (sequential).
[[nodiscard]] std::vector<Contact> detectCollisionsSpatialGrid(
    const Engine& engine,
    float cellSize = SpatialGrid::kDefaultCellSize);

/// Standalone convenience wrapper for parallel spatial grid collision detection.
void detectCollisionsSpatialGridParallel(const Engine& engine,
                                        std::vector<Contact>& outContacts,
                                        WorkerPool& pool,
                                        float cellSize = SpatialGrid::kDefaultCellSize);

/// Standalone convenience wrapper returning a vector of contacts (parallel).
[[nodiscard]] std::vector<Contact> detectCollisionsSpatialGridParallel(
    const Engine& engine,
    WorkerPool& pool,
    float cellSize = SpatialGrid::kDefaultCellSize);

} // namespace PhysicsEngine
