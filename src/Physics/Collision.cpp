// Collision.cpp -- Narrow-phase collision tests and brute-force pair generator.
//
// See Collision.h for design rationale, conventions, and invariants.

#include "PhysicsEngine/Physics/Collision.h"

#include <algorithm>
#include <cmath>

namespace PhysicsEngine {

// ─────────────────────────────────────────────────────────────────────────────
// Geometric Narrowphase Tests
// ─────────────────────────────────────────────────────────────────────────────

bool testCircleCircle(Vec2 posA, float radiusA,
                      Vec2 posB, float radiusB,
                      Contact& outContact,
                      BodyHandle bodyA,
                      BodyHandle bodyB) noexcept
{
    // Reject invalid dimensions or non-finite coordinates
    if (!std::isfinite(posA.x) || !std::isfinite(posA.y) ||
        !std::isfinite(posB.x) || !std::isfinite(posB.y) ||
        !std::isfinite(radiusA) || !std::isfinite(radiusB) ||
        radiusA <= 0.f || radiusB <= 0.f) {
        return false;
    }

    const Vec2 delta = posB - posA;
    const float distSq = delta.lengthSq();
    const float radiusSum = radiusA + radiusB;

    // Touching boundaries (distSq == radiusSum * radiusSum) do NOT count as collision
    if (distSq >= radiusSum * radiusSum) {
        return false;
    }

    constexpr float kEpsilonSq = 1e-12f;
    Vec2 normal;
    float dist = 0.f;

    if (distSq > kEpsilonSq) {
        dist = std::sqrt(distSq);
        normal = delta / dist; // Points from A toward B
    } else {
        // Coincident circle centers: choose deterministic unit normal (0, 1)
        dist = 0.f;
        normal = Vec2{0.f, 1.f};
    }

    outContact.bodyA       = bodyA;
    outContact.bodyB       = bodyB;
    outContact.normal      = normal;
    outContact.penetration = radiusSum - dist;
    // Midpoint between surfaces along normal
    outContact.point       = posA + normal * (radiusA - 0.5f * outContact.penetration);

    return true;
}

bool testAABBAABB(Vec2 posA, Vec2 halfExtentsA,
                  Vec2 posB, Vec2 halfExtentsB,
                  Contact& outContact,
                  BodyHandle bodyA,
                  BodyHandle bodyB) noexcept
{
    // Reject invalid dimensions or non-finite coordinates
    if (!std::isfinite(posA.x) || !std::isfinite(posA.y) ||
        !std::isfinite(posB.x) || !std::isfinite(posB.y) ||
        !std::isfinite(halfExtentsA.x) || !std::isfinite(halfExtentsA.y) ||
        !std::isfinite(halfExtentsB.x) || !std::isfinite(halfExtentsB.y) ||
        halfExtentsA.x <= 0.f || halfExtentsA.y <= 0.f ||
        halfExtentsB.x <= 0.f || halfExtentsB.y <= 0.f) {
        return false;
    }

    const Vec2 delta = posB - posA;
    // Minimum translation distance (MTD) along each axis.
    // Handles both partial overlap and full containment correctly.
    const float overlapX = (halfExtentsA.x + halfExtentsB.x) - std::abs(delta.x);
    const float overlapY = (halfExtentsA.y + halfExtentsB.y) - std::abs(delta.y);

    // Strict overlap required on both axes (touching boundary <= 0 does NOT collide)
    if (overlapX <= 0.f || overlapY <= 0.f) {
        return false;
    }

    outContact.bodyA = bodyA;
    outContact.bodyB = bodyB;

    // Minimum separating axis (SAT)
    if (overlapX < overlapY) {
        outContact.penetration = overlapX;
        // Normal points from A toward B
        const float signX = (delta.x >= 0.f) ? 1.f : -1.f;
        outContact.normal = Vec2{signX, 0.f};

        // Contact point on overlap interface
        const float contactX = posA.x + signX * halfExtentsA.x;
        const float minY = std::max(posA.y - halfExtentsA.y, posB.y - halfExtentsB.y);
        const float maxY = std::min(posA.y + halfExtentsA.y, posB.y + halfExtentsB.y);
        outContact.point = Vec2{contactX, 0.5f * (minY + maxY)};
    } else {
        outContact.penetration = overlapY;
        const float signY = (delta.y >= 0.f) ? 1.f : -1.f;
        outContact.normal = Vec2{0.f, signY};

        const float contactY = posA.y + signY * halfExtentsA.y;
        const float minX = std::max(posA.x - halfExtentsA.x, posB.x - halfExtentsB.x);
        const float maxX = std::min(posA.x + halfExtentsA.x, posB.x + halfExtentsB.x);
        outContact.point = Vec2{0.5f * (minX + maxX), contactY};
    }

    return true;
}

bool testCircleAABB(Vec2 circlePos, float radius,
                    Vec2 aabbPos, Vec2 aabbHalfExtents,
                    Contact& outContact,
                    BodyHandle bodyA,
                    BodyHandle bodyB) noexcept
{
    // Reject invalid dimensions or non-finite coordinates
    if (!std::isfinite(circlePos.x) || !std::isfinite(circlePos.y) ||
        !std::isfinite(aabbPos.x) || !std::isfinite(aabbPos.y) ||
        !std::isfinite(radius) ||
        !std::isfinite(aabbHalfExtents.x) || !std::isfinite(aabbHalfExtents.y) ||
        radius <= 0.f || aabbHalfExtents.x <= 0.f || aabbHalfExtents.y <= 0.f) {
        return false;
    }

    // Find closest point on AABB to circle center
    const Vec2 bMin = aabbPos - aabbHalfExtents;
    const Vec2 bMax = aabbPos + aabbHalfExtents;
    const Vec2 closestPoint{
        std::clamp(circlePos.x, bMin.x, bMax.x),
        std::clamp(circlePos.y, bMin.y, bMax.y)
    };

    const Vec2 delta = closestPoint - circlePos; // from circle toward closest point
    const float distSq = delta.lengthSq();

    // Case 1: Circle center is strictly outside the AABB
    if (distSq > 0.f) {
        if (distSq >= radius * radius) {
            return false; // separated or touching
        }
        const float dist = std::sqrt(distSq);
        const Vec2 normal = delta / dist; // Points from circle A toward AABB B

        outContact.bodyA       = bodyA;
        outContact.bodyB       = bodyB;
        outContact.normal      = normal;
        outContact.penetration = radius - dist;
        outContact.point       = closestPoint;
        return true;
    }

    // Case 2: Circle center is inside or on the boundary of the AABB.
    // To separate circle (bodyA) from AABB (bodyB), the circle must exit
    // through the nearest AABB face.
    // Since positional resolution applies -normal to bodyA and +normal to bodyB:
    //   - If nearest face is Right (+X): Circle must move +X -> normal = (-1, 0)
    //   - If nearest face is Left (-X):  Circle must move -X -> normal = (+1, 0)
    //   - If nearest face is Top (+Y):   Circle must move +Y -> normal = (0, -1)
    //   - If nearest face is Bottom (-Y):Circle must move -Y -> normal = (0, +1)
    // This convention maintains continuity across the AABB surface boundary.
    const float dxRight  = bMax.x - circlePos.x;
    const float dxLeft   = circlePos.x - bMin.x;
    const float dyTop    = bMax.y - circlePos.y;
    const float dyBottom = circlePos.y - bMin.y;

    float minDistance = dxRight;
    Vec2 normal{-1.f, 0.f};
    Vec2 contactPoint{bMax.x, circlePos.y};

    if (dxLeft < minDistance) {
        minDistance = dxLeft;
        normal = Vec2{1.f, 0.f};
        contactPoint = Vec2{bMin.x, circlePos.y};
    }
    if (dyTop < minDistance) {
        minDistance = dyTop;
        normal = Vec2{0.f, -1.f};
        contactPoint = Vec2{circlePos.x, bMax.y};
    }
    if (dyBottom < minDistance) {
        minDistance = dyBottom;
        normal = Vec2{0.f, 1.f};
        contactPoint = Vec2{circlePos.x, bMin.y};
    }

    outContact.bodyA       = bodyA;
    outContact.bodyB       = bodyB;
    outContact.normal      = normal;
    outContact.penetration = radius + minDistance;
    outContact.point       = contactPoint;

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Handle-based & Slot-based Dispatchers
// ─────────────────────────────────────────────────────────────────────────────

bool testCollisionSlots(const Engine& engine,
                        uint32_t slotA,
                        uint32_t slotB,
                        Contact& outContact,
                        BodyHandle handleA,
                        BodyHandle handleB) noexcept
{
    if (slotA == slotB || handleA == handleB) {
        return false;
    }

    const uint32_t count = engine.bodyCount();
    if (slotA >= count || slotB >= count) {
        return false;
    }

    if (handleA.isNull() || handleB.isNull()) {
        return false;
    }

    const ShapeType* shapeTypes  = engine.shapeTypesData();
    const Vec2*      positions   = engine.positionsData();
    const float*     radii       = engine.radiiData();
    const float*     halfHeights = engine.halfHeightsData();

    const ShapeType typeA = shapeTypes[slotA];
    const ShapeType typeB = shapeTypes[slotB];
    const Vec2      posA  = positions[slotA];
    const Vec2      posB  = positions[slotB];

    if (typeA == ShapeType::Circle && typeB == ShapeType::Circle) {
        return testCircleCircle(posA, radii[slotA],
                                posB, radii[slotB],
                                outContact, handleA, handleB);
    }

    if (typeA == ShapeType::AABB && typeB == ShapeType::AABB) {
        return testAABBAABB(posA, Vec2{radii[slotA], halfHeights[slotA]},
                            posB, Vec2{radii[slotB], halfHeights[slotB]},
                            outContact, handleA, handleB);
    }

    if (typeA == ShapeType::Circle && typeB == ShapeType::AABB) {
        return testCircleAABB(posA, radii[slotA],
                              posB, Vec2{radii[slotB], halfHeights[slotB]},
                              outContact, handleA, handleB);
    }

    if (typeA == ShapeType::AABB && typeB == ShapeType::Circle) {
        Contact contactBA;
        if (testCircleAABB(posB, radii[slotB],
                           posA, Vec2{radii[slotA], halfHeights[slotA]},
                           contactBA, handleB, handleA)) {
            outContact.bodyA       = handleA;
            outContact.bodyB       = handleB;
            outContact.normal      = -contactBA.normal; // Points A -> B
            outContact.penetration = contactBA.penetration;
            outContact.point       = contactBA.point;
            return true;
        }
        return false;
    }

    return false;
}

bool testCollision(const Engine& engine,
                   BodyHandle handleA,
                   BodyHandle handleB,
                   Contact& outContact) noexcept
{
    // Never collide with self
    if (handleA == handleB) {
        return false;
    }

    // Validate handles
    if (!engine.isValidHandle(handleA) || !engine.isValidHandle(handleB)) {
        return false;
    }

    const ShapeType typeA = engine.getShapeType(handleA);
    const ShapeType typeB = engine.getShapeType(handleB);
    const Vec2 posA = engine.getPosition(handleA);
    const Vec2 posB = engine.getPosition(handleB);

    if (typeA == ShapeType::Circle && typeB == ShapeType::Circle) {
        return testCircleCircle(posA, engine.getRadius(handleA),
                                posB, engine.getRadius(handleB),
                                outContact, handleA, handleB);
    }

    if (typeA == ShapeType::AABB && typeB == ShapeType::AABB) {
        const Vec2 extA{engine.getRadius(handleA), engine.getHalfHeight(handleA)};
        const Vec2 extB{engine.getRadius(handleB), engine.getHalfHeight(handleB)};
        return testAABBAABB(posA, extA, posB, extB, outContact, handleA, handleB);
    }

    if (typeA == ShapeType::Circle && typeB == ShapeType::AABB) {
        const Vec2 extB{engine.getRadius(handleB), engine.getHalfHeight(handleB)};
        return testCircleAABB(posA, engine.getRadius(handleA),
                              posB, extB, outContact, handleA, handleB);
    }

    if (typeA == ShapeType::AABB && typeB == ShapeType::Circle) {
        const Vec2 extA{engine.getRadius(handleA), engine.getHalfHeight(handleA)};
        Contact contactBA;
        if (testCircleAABB(posB, engine.getRadius(handleB),
                           posA, extA, contactBA, handleB, handleA)) {
            outContact.bodyA       = handleA;
            outContact.bodyB       = handleB;
            outContact.normal      = -contactBA.normal; // Points A -> B
            outContact.penetration = contactBA.penetration;
            outContact.point       = contactBA.point;
            return true;
        }
        return false;
    }

    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Reference Brute-force Pair Generation
// ─────────────────────────────────────────────────────────────────────────────

void detectCollisionsBruteForce(const Engine& engine, std::vector<Contact>& outContacts)
{
    outContacts.clear();
    const uint32_t count = engine.bodyCount();
    if (count < 2) {
        return;
    }

    const float* inverseMasses = engine.inverseMassesData();

    Contact contact;
    for (uint32_t i = 0; i < count; ++i) {
        const bool isStaticA = (inverseMasses[i] == 0.f);

        for (uint32_t j = i + 1; j < count; ++j) {
            // Omit static-static pairs: static bodies never move, so resolving
            // contacts between them is physically unnecessary.
            if (isStaticA && (inverseMasses[j] == 0.f)) {
                continue;
            }

            const BodyHandle handleA = engine.handleFromSlot(i);
            const BodyHandle handleB = engine.handleFromSlot(j);

            if (testCollisionSlots(engine, i, j, contact, handleA, handleB)) {
                outContacts.push_back(contact);
            }
        }
    }
}

std::vector<Contact> detectCollisionsBruteForce(const Engine& engine)
{
    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    return contacts;
}

} // namespace PhysicsEngine
