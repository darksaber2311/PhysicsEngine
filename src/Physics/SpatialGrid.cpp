// SpatialGrid.cpp -- Uniform spatial grid broad-phase collision detection.
//
// See SpatialGrid.h for design rationale, invariants, and contract.

#include "PhysicsEngine/Physics/SpatialGrid.h"

#include <algorithm>
#include <cmath>

namespace PhysicsEngine {

SpatialGrid::SpatialGrid(float cellSize) noexcept
{
    setCellSize(cellSize);
}

SpatialGrid::SpatialGrid(const SpatialGridConfig& config) noexcept
    : sequentialThreshold_(config.sequentialThreshold)
{
    setCellSize(config.cellSize);
}

void SpatialGrid::setCellSize(float cellSize) noexcept
{
    if (cellSize > 0.f && std::isfinite(cellSize)) {
        cellSize_ = cellSize;
    } else {
        cellSize_ = kDefaultCellSize;
    }
}

void SpatialGrid::clear() noexcept
{
    entries_.clear();
    candidatePairs_.clear();
    oversizedBodies_.clear();
    workerBuffers_.clear();
    chunkContacts_.clear();
    lastCandidatePairCount_ = 0;
}

void SpatialGrid::detectCollisions(const Engine& engine, std::vector<Contact>& outContacts)
{
    outContacts.clear();
    entries_.clear();
    candidatePairs_.clear();
    oversizedBodies_.clear();
    lastCandidatePairCount_ = 0;

    const uint32_t count = engine.bodyCount();
    if (count < 2) {
        return;
    }

    const Vec2*      positions     = engine.positionsData();
    const float*     inverseMasses = engine.inverseMassesData();
    const ShapeType* shapeTypes    = engine.shapeTypesData();
    const float*     radii         = engine.radiiData();
    const float*     halfHeights   = engine.halfHeightsData();

    const float invCellSize = 1.0f / cellSize_;

    // ── Phase 1: Compute conservative AABB and insert into overlapping cells ─
    for (uint32_t i = 0; i < count; ++i) {
        const Vec2      pos  = positions[i];
        const ShapeType type = shapeTypes[i];
        const float     hw   = radii[i];
        const float     hh   = (type == ShapeType::AABB) ? halfHeights[i] : hw;

        // Numeric validation: skip bodies with invalid dimensions or non-finite positions
        if (!std::isfinite(pos.x) || !std::isfinite(pos.y) ||
            !std::isfinite(hw) || !std::isfinite(hh) ||
            hw <= 0.f || hh <= 0.f) {
            continue;
        }

        // Coordinate range validation: skip bodies outside supported coordinate space
        if (std::abs(pos.x) > kMaxCoord || std::abs(pos.y) > kMaxCoord ||
            hw > kMaxCoord || hh > kMaxCoord) {
            continue;
        }

        const Vec2 minExt = pos - Vec2{hw, hh};
        const Vec2 maxExt = pos + Vec2{hw, hh};

        // Floor-based indexing for negative coordinates
        const int32_t minCellX = static_cast<int32_t>(std::floor(minExt.x * invCellSize));
        const int32_t maxCellX = static_cast<int32_t>(std::floor(maxExt.x * invCellSize));
        const int32_t minCellY = static_cast<int32_t>(std::floor(minExt.y * invCellSize));
        const int32_t maxCellY = static_cast<int32_t>(std::floor(maxExt.y * invCellSize));

        const int64_t spanX = static_cast<int64_t>(maxCellX) - static_cast<int64_t>(minCellX) + 1;
        const int64_t spanY = static_cast<int64_t>(maxCellY) - static_cast<int64_t>(minCellY) + 1;
        if (spanX <= 0 || spanY <= 0) {
            continue;
        }

        const int64_t totalCells = spanX * spanY;

        // If body spans more than kMaxCellsPerBody cells, handle via oversized fallback
        // to prevent flooding grid storage while preserving exact collision detection
        if (totalCells > kMaxCellsPerBody) {
            oversizedBodies_.push_back(i);
        } else {
            for (int32_t cy = minCellY; cy <= maxCellY; ++cy) {
                for (int32_t cx = minCellX; cx <= maxCellX; ++cx) {
                    const uint64_t cellKey = (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) |
                                              static_cast<uint64_t>(static_cast<uint32_t>(cy));
                    entries_.push_back(CellEntry{cellKey, i});
                }
            }
        }
    }

    // ── Phase 2: Sort entries by (cellKey, bodySlot) ─────────────────────────
    std::sort(entries_.begin(), entries_.end());

    // ── Phase 3A: Gather candidate pairs from shared cells ───────────────────
    const size_t entryCount = entries_.size();
    size_t cellStart = 0;

    while (cellStart < entryCount) {
        size_t cellEnd = cellStart + 1;
        const uint64_t currentKey = entries_[cellStart].cellKey;
        while (cellEnd < entryCount && entries_[cellEnd].cellKey == currentKey) {
            ++cellEnd;
        }

        const size_t countInCell = cellEnd - cellStart;
        if (countInCell >= 2) {
            for (size_t p = cellStart; p < cellEnd; ++p) {
                const uint32_t slotA = entries_[p].bodySlot;
                const bool isStaticA = (inverseMasses[slotA] == 0.f);

                for (size_t q = p + 1; q < cellEnd; ++q) {
                    const uint32_t slotB = entries_[q].bodySlot;

                    // Omit static-static pairs
                    if (isStaticA && (inverseMasses[slotB] == 0.f)) {
                        continue;
                    }

                    // Since entries are sorted by (cellKey, bodySlot), slotA < slotB
                    const uint64_t pairKey = (static_cast<uint64_t>(slotA) << 32) |
                                              static_cast<uint64_t>(slotB);
                    candidatePairs_.push_back(pairKey);
                }
            }
        }

        cellStart = cellEnd;
    }

    // ── Phase 3B: Oversized bodies fallback testing ─────────────────────────
    // Tests oversized bodies against all other active bodies using fast AABB overlap
    if (!oversizedBodies_.empty()) {
        for (uint32_t oSlot : oversizedBodies_) {
            const Vec2      oPos  = positions[oSlot];
            const ShapeType oType = shapeTypes[oSlot];
            const float     oHw   = radii[oSlot];
            const float     oHh   = (oType == ShapeType::AABB) ? halfHeights[oSlot] : oHw;
            const Vec2      oMin  = oPos - Vec2{oHw, oHh};
            const Vec2      oMax  = oPos + Vec2{oHw, oHh};
            const bool      isStaticO = (inverseMasses[oSlot] == 0.f);

            for (uint32_t bSlot = 0; bSlot < count; ++bSlot) {
                if (bSlot == oSlot) {
                    continue;
                }

                // Omit static-static pairs
                if (isStaticO && (inverseMasses[bSlot] == 0.f)) {
                    continue;
                }

                const Vec2      bPos  = positions[bSlot];
                const ShapeType bType = shapeTypes[bSlot];
                const float     bHw   = radii[bSlot];
                const float     bHh   = (bType == ShapeType::AABB) ? halfHeights[bSlot] : bHw;

                if (!std::isfinite(bPos.x) || !std::isfinite(bPos.y) || bHw <= 0.f || bHh <= 0.f) {
                    continue;
                }

                const Vec2 bMin = bPos - Vec2{bHw, bHh};
                const Vec2 bMax = bPos + Vec2{bHw, bHh};

                // Conservative AABB overlap test
                if (bMax.x >= oMin.x && bMin.x <= oMax.x &&
                    bMax.y >= oMin.y && bMin.y <= oMax.y) {
                    const uint32_t slotA = std::min(oSlot, bSlot);
                    const uint32_t slotB = std::max(oSlot, bSlot);
                    const uint64_t pairKey = (static_cast<uint64_t>(slotA) << 32) |
                                              static_cast<uint64_t>(slotB);
                    candidatePairs_.push_back(pairKey);
                }
            }
        }
    }

    // ── Phase 4: Deduplicate candidate pairs ──────────────────────────────────
    std::sort(candidatePairs_.begin(), candidatePairs_.end());
    candidatePairs_.erase(std::unique(candidatePairs_.begin(), candidatePairs_.end()),
                          candidatePairs_.end());
    lastCandidatePairCount_ = candidatePairs_.size();

    // ── Phase 5: Narrow-phase validation and contact emission ────────────────
    Contact contact;
    for (uint64_t pairKey : candidatePairs_) {
        const uint32_t slotA = static_cast<uint32_t>(pairKey >> 32);
        const uint32_t slotB = static_cast<uint32_t>(pairKey & 0xFFFFFFFFULL);

        const BodyHandle handleA = engine.handleFromSlot(slotA);
        const BodyHandle handleB = engine.handleFromSlot(slotB);

        if (testCollisionSlots(engine, slotA, slotB, contact, handleA, handleB)) {
            outContacts.push_back(contact);
        }
    }
}

std::vector<Contact> SpatialGrid::detectCollisions(const Engine& engine)
{
    std::vector<Contact> contacts;
    detectCollisions(engine, contacts);
    return contacts;
}

void SpatialGrid::detectCollisionsParallel(const Engine& engine,
                                          std::vector<Contact>& outContacts,
                                          WorkerPool& pool)
{
    outContacts.clear();
    lastCandidatePairCount_ = 0;

    const uint32_t count = engine.bodyCount();
    if (count < 2) {
        return;
    }

    // Fall back to sequential execution if body count is below threshold or pool has 0 workers
    if (count < sequentialThreshold_ || pool.workerCount() == 0) {
        detectCollisions(engine, outContacts);
        return;
    }

    entries_.clear();
    candidatePairs_.clear();
    oversizedBodies_.clear();

    const Vec2*      positions     = engine.positionsData();
    const float*     inverseMasses = engine.inverseMassesData();
    const ShapeType* shapeTypes    = engine.shapeTypesData();
    const float*     radii         = engine.radiiData();
    const float*     halfHeights   = engine.halfHeightsData();

    const float invCellSize = 1.0f / cellSize_;
    const uint32_t numWorkers = pool.workerCount() + 1; // background threads + calling thread

    if (workerBuffers_.size() < numWorkers) {
        workerBuffers_.resize(numWorkers);
    }
    for (uint32_t w = 0; w < numWorkers; ++w) {
        workerBuffers_[w].entries.clear();
        workerBuffers_[w].oversized.clear();
        workerBuffers_[w].candidatePairs.clear();
    }

    // ── Phase 1: Parallel conservative AABB & cell insertion ─────────────────
    pool.parallelFor(count, [&](size_t start, size_t end, uint32_t workerIndex) {
        auto& localEntries = workerBuffers_[workerIndex].entries;
        auto& localOversized = workerBuffers_[workerIndex].oversized;

        for (size_t i = start; i < end; ++i) {
            const Vec2      pos  = positions[i];
            const ShapeType type = shapeTypes[i];
            const float     hw   = radii[i];
            const float     hh   = (type == ShapeType::AABB) ? halfHeights[i] : hw;

            if (!std::isfinite(pos.x) || !std::isfinite(pos.y) ||
                !std::isfinite(hw) || !std::isfinite(hh) ||
                hw <= 0.f || hh <= 0.f) {
                continue;
            }

            if (std::abs(pos.x) > kMaxCoord || std::abs(pos.y) > kMaxCoord ||
                hw > kMaxCoord || hh > kMaxCoord) {
                continue;
            }

            const Vec2 minExt = pos - Vec2{hw, hh};
            const Vec2 maxExt = pos + Vec2{hw, hh};

            const int32_t minCellX = static_cast<int32_t>(std::floor(minExt.x * invCellSize));
            const int32_t maxCellX = static_cast<int32_t>(std::floor(maxExt.x * invCellSize));
            const int32_t minCellY = static_cast<int32_t>(std::floor(minExt.y * invCellSize));
            const int32_t maxCellY = static_cast<int32_t>(std::floor(maxExt.y * invCellSize));

            const int64_t spanX = static_cast<int64_t>(maxCellX) - static_cast<int64_t>(minCellX) + 1;
            const int64_t spanY = static_cast<int64_t>(maxCellY) - static_cast<int64_t>(minCellY) + 1;
            if (spanX <= 0 || spanY <= 0) {
                continue;
            }

            const int64_t totalCells = spanX * spanY;
            if (totalCells > kMaxCellsPerBody) {
                localOversized.push_back(static_cast<uint32_t>(i));
            } else {
                for (int32_t cy = minCellY; cy <= maxCellY; ++cy) {
                    for (int32_t cx = minCellX; cx <= maxCellX; ++cx) {
                        const uint64_t cellKey = (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) |
                                                  static_cast<uint64_t>(static_cast<uint32_t>(cy));
                        localEntries.push_back(CellEntry{cellKey, static_cast<uint32_t>(i)});
                    }
                }
            }
        }
    }, /*minChunkSize=*/ 128);

    // Merge entries and oversized bodies
    size_t totalEntries = 0;
    for (uint32_t w = 0; w < numWorkers; ++w) {
        totalEntries += workerBuffers_[w].entries.size();
    }
    entries_.resize(totalEntries);

    size_t offset = 0;
    for (uint32_t w = 0; w < numWorkers; ++w) {
        const auto& e = workerBuffers_[w].entries;
        if (!e.empty()) {
            std::copy(e.begin(), e.end(), entries_.begin() + offset);
            offset += e.size();
        }
        const auto& o = workerBuffers_[w].oversized;
        if (!o.empty()) {
            oversizedBodies_.insert(oversizedBodies_.end(), o.begin(), o.end());
        }
    }

    // ── Phase 2: Sort entries by (cellKey, bodySlot) ─────────────────────────
    std::sort(entries_.begin(), entries_.end());

    // ── Phase 3A: Parallel Candidate Generation from Shared Cells ────────────
    const size_t entryCount = entries_.size();
    if (entryCount >= 2) {
        // Partition entryCount into slices where slice boundaries never split a cell
        const size_t sliceCount = std::min<size_t>(numWorkers, (entryCount + 255) / 256);
        std::vector<size_t> sliceBoundaries(sliceCount + 1, 0);
        sliceBoundaries[0] = 0;
        sliceBoundaries[sliceCount] = entryCount;

        for (size_t s = 1; s < sliceCount; ++s) {
            size_t nominal = (s * entryCount) / sliceCount;
            while (nominal < entryCount && entries_[nominal].cellKey == entries_[nominal - 1].cellKey) {
                ++nominal;
            }
            sliceBoundaries[s] = nominal;
        }

        pool.parallelFor(sliceCount, [&](size_t start, size_t end, uint32_t workerIndex) {
            auto& localPairs = workerBuffers_[workerIndex].candidatePairs;

            for (size_t s = start; s < end; ++s) {
                size_t cellStart = sliceBoundaries[s];
                const size_t sliceEnd = sliceBoundaries[s + 1];

                while (cellStart < sliceEnd) {
                    size_t cellEnd = cellStart + 1;
                    const uint64_t currentKey = entries_[cellStart].cellKey;
                    while (cellEnd < entryCount && entries_[cellEnd].cellKey == currentKey) {
                        ++cellEnd;
                    }

                    const size_t countInCell = cellEnd - cellStart;
                    if (countInCell >= 2) {
                        for (size_t p = cellStart; p < cellEnd; ++p) {
                            const uint32_t slotA = entries_[p].bodySlot;
                            const bool isStaticA = (inverseMasses[slotA] == 0.f);

                            for (size_t q = p + 1; q < cellEnd; ++q) {
                                const uint32_t slotB = entries_[q].bodySlot;
                                if (isStaticA && (inverseMasses[slotB] == 0.f)) {
                                    continue;
                                }

                                const uint64_t pairKey = (static_cast<uint64_t>(slotA) << 32) |
                                                          static_cast<uint64_t>(slotB);
                                localPairs.push_back(pairKey);
                            }
                        }
                    }

                    cellStart = cellEnd;
                }
            }
        }, 1);
    }

    // ── Phase 3B: Parallel Oversized Bodies Fallback Testing ──────────────────
    if (!oversizedBodies_.empty()) {
        pool.parallelFor(oversizedBodies_.size(), [&](size_t start, size_t end, uint32_t workerIndex) {
            auto& localPairs = workerBuffers_[workerIndex].candidatePairs;

            for (size_t oIdx = start; oIdx < end; ++oIdx) {
                const uint32_t  oSlot     = oversizedBodies_[oIdx];
                const Vec2      oPos      = positions[oSlot];
                const ShapeType oType     = shapeTypes[oSlot];
                const float     oHw       = radii[oSlot];
                const float     oHh       = (oType == ShapeType::AABB) ? halfHeights[oSlot] : oHw;
                const Vec2      oMin      = oPos - Vec2{oHw, oHh};
                const Vec2      oMax      = oPos + Vec2{oHw, oHh};
                const bool      isStaticO = (inverseMasses[oSlot] == 0.f);

                for (uint32_t bSlot = 0; bSlot < count; ++bSlot) {
                    if (bSlot == oSlot) {
                        continue;
                    }
                    if (isStaticO && (inverseMasses[bSlot] == 0.f)) {
                        continue;
                    }

                    const Vec2      bPos  = positions[bSlot];
                    const ShapeType bType = shapeTypes[bSlot];
                    const float     bHw   = radii[bSlot];
                    const float     bHh   = (bType == ShapeType::AABB) ? halfHeights[bSlot] : bHw;

                    if (!std::isfinite(bPos.x) || !std::isfinite(bPos.y) || bHw <= 0.f || bHh <= 0.f) {
                        continue;
                    }

                    const Vec2 bMin = bPos - Vec2{bHw, bHh};
                    const Vec2 bMax = bPos + Vec2{bHw, bHh};

                    if (bMax.x >= oMin.x && bMin.x <= oMax.x &&
                        bMax.y >= oMin.y && bMin.y <= oMax.y) {
                        const uint32_t slotA = std::min(oSlot, bSlot);
                        const uint32_t slotB = std::max(oSlot, bSlot);
                        const uint64_t pairKey = (static_cast<uint64_t>(slotA) << 32) |
                                                  static_cast<uint64_t>(slotB);
                        localPairs.push_back(pairKey);
                    }
                }
            }
        }, 1);
    }

    // ── Phase 4: Deduplicate Candidate Pairs ──────────────────────────────────
    size_t totalCandidatePairs = 0;
    for (uint32_t w = 0; w < numWorkers; ++w) {
        totalCandidatePairs += workerBuffers_[w].candidatePairs.size();
    }
    candidatePairs_.resize(totalCandidatePairs);

    size_t pOffset = 0;
    for (uint32_t w = 0; w < numWorkers; ++w) {
        const auto& p = workerBuffers_[w].candidatePairs;
        if (!p.empty()) {
            std::copy(p.begin(), p.end(), candidatePairs_.begin() + pOffset);
            pOffset += p.size();
        }
    }

    std::sort(candidatePairs_.begin(), candidatePairs_.end());
    candidatePairs_.erase(std::unique(candidatePairs_.begin(), candidatePairs_.end()),
                          candidatePairs_.end());
    lastCandidatePairCount_ = candidatePairs_.size();

    // ── Phase 5: Deterministic Parallel Narrow-Phase Dispatch ─────────────────
    const size_t pairCount = candidatePairs_.size();
    if (pairCount == 0) {
        return;
    }

    // For very few pairs, run sequentially directly to avoid chunk setup overhead
    if (pairCount < 64) {
        Contact contact;
        for (uint64_t pairKey : candidatePairs_) {
            const uint32_t slotA = static_cast<uint32_t>(pairKey >> 32);
            const uint32_t slotB = static_cast<uint32_t>(pairKey & 0xFFFFFFFFULL);

            const BodyHandle handleA = engine.handleFromSlot(slotA);
            const BodyHandle handleB = engine.handleFromSlot(slotB);

            if (testCollisionSlots(engine, slotA, slotB, contact, handleA, handleB)) {
                outContacts.push_back(contact);
            }
        }
        return;
    }

    // Deterministic chunked narrow-phase:
    // Each chunk c writes to chunkContacts_[c].
    // Chunks are merged strictly in index order 0, 1, 2, ... to preserve 100% bit-for-bit contact ordering.
    const size_t numChunks = std::min<size_t>(numWorkers * 2, (pairCount + 31) / 32);
    if (chunkContacts_.size() < numChunks) {
        chunkContacts_.resize(numChunks);
    }
    for (size_t c = 0; c < numChunks; ++c) {
        chunkContacts_[c].clear();
    }

    const size_t chunkSize = (pairCount + numChunks - 1) / numChunks;

    pool.parallelFor(numChunks, [&](size_t start, size_t end, uint32_t /*workerIndex*/) {
        Contact contact;
        for (size_t c = start; c < end; ++c) {
            const size_t pStart = c * chunkSize;
            const size_t pEnd   = std::min(pairCount, pStart + chunkSize);

            for (size_t p = pStart; p < pEnd; ++p) {
                const uint64_t pairKey = candidatePairs_[p];
                const uint32_t slotA = static_cast<uint32_t>(pairKey >> 32);
                const uint32_t slotB = static_cast<uint32_t>(pairKey & 0xFFFFFFFFULL);

                const BodyHandle handleA = engine.handleFromSlot(slotA);
                const BodyHandle handleB = engine.handleFromSlot(slotB);

                if (testCollisionSlots(engine, slotA, slotB, contact, handleA, handleB)) {
                    chunkContacts_[c].push_back(contact);
                }
            }
        }
    }, 1);

    // Merge in deterministic chunk order
    size_t totalContacts = 0;
    for (size_t c = 0; c < numChunks; ++c) {
        totalContacts += chunkContacts_[c].size();
    }
    outContacts.reserve(totalContacts);
    for (size_t c = 0; c < numChunks; ++c) {
        const auto& cc = chunkContacts_[c];
        if (!cc.empty()) {
            outContacts.insert(outContacts.end(), cc.begin(), cc.end());
        }
    }
}

std::vector<Contact> SpatialGrid::detectCollisionsParallel(const Engine& engine, WorkerPool& pool)
{
    std::vector<Contact> contacts;
    detectCollisionsParallel(engine, contacts, pool);
    return contacts;
}

void detectCollisionsSpatialGrid(const Engine& engine,
                                std::vector<Contact>& outContacts,
                                float cellSize)
{
    SpatialGrid grid(cellSize);
    grid.detectCollisions(engine, outContacts);
}

std::vector<Contact> detectCollisionsSpatialGrid(const Engine& engine, float cellSize)
{
    std::vector<Contact> contacts;
    detectCollisionsSpatialGrid(engine, contacts, cellSize);
    return contacts;
}

void detectCollisionsSpatialGridParallel(const Engine& engine,
                                        std::vector<Contact>& outContacts,
                                        WorkerPool& pool,
                                        float cellSize)
{
    SpatialGrid grid(cellSize);
    grid.detectCollisionsParallel(engine, outContacts, pool);
}

std::vector<Contact> detectCollisionsSpatialGridParallel(
    const Engine& engine,
    WorkerPool& pool,
    float cellSize)
{
    std::vector<Contact> contacts;
    detectCollisionsSpatialGridParallel(engine, contacts, pool, cellSize);
    return contacts;
}

} // namespace PhysicsEngine
