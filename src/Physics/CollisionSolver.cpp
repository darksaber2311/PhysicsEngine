// CollisionSolver.cpp -- Impulse-based collision response and contact resolution.
//
// See CollisionSolver.h for architectural contract and design invariants.

#include "PhysicsEngine/Physics/CollisionSolver.h"

#include <algorithm>
#include <cmath>

namespace PhysicsEngine {

namespace {

float clamp01(float v) noexcept
{
    if (v < 0.f) return 0.f;
    if (v > 1.f) return 1.f;
    return v;
}

} // anonymous namespace

void resolveVelocityImpulses(Engine& engine, std::span<const Contact> contacts)
{
    for (const Contact& contact : contacts) {
        if (contact.bodyA == contact.bodyB) {
            continue;
        }

        if (!engine.isValidHandle(contact.bodyA) || !engine.isValidHandle(contact.bodyB)) {
            continue;
        }

        const float invMassA = engine.getInverseMass(contact.bodyA);
        const float invMassB = engine.getInverseMass(contact.bodyB);
        const float totalInvMass = invMassA + invMassB;

        // Skip static-static contacts or invalid non-positive inverse masses
        if (totalInvMass <= 0.f || !std::isfinite(totalInvMass)) {
            continue;
        }

        const Vec2 velA   = engine.getVelocity(contact.bodyA);
        const Vec2 velB   = engine.getVelocity(contact.bodyB);
        const Vec2 normal = contact.normal;

        if (!std::isfinite(velA.x) || !std::isfinite(velA.y) ||
            !std::isfinite(velB.x) || !std::isfinite(velB.y) ||
            !std::isfinite(normal.x) || !std::isfinite(normal.y)) {
            continue;
        }

        // Relative velocity: velocity of B relative to A
        const Vec2 relVel = velB - velA;
        const float normalVel = Vec2::dot(relVel, normal);

        // If bodies are separating or stationary in the normal direction, do not apply impulse
        if (normalVel >= 0.f) {
            continue;
        }

        const float eA = engine.getRestitution(contact.bodyA);
        const float eB = engine.getRestitution(contact.bodyB);
        const float restitution = std::min(
            clamp01(std::isfinite(eA) ? eA : 0.f),
            clamp01(std::isfinite(eB) ? eB : 0.f)
        );

        const float j = -(1.f + restitution) * normalVel / totalInvMass;
        if (!std::isfinite(j)) {
            continue;
        }

        const Vec2 impulse = normal * j;

        if (invMassA > 0.f) {
            engine.setVelocity(contact.bodyA, velA - impulse * invMassA);
        }
        if (invMassB > 0.f) {
            engine.setVelocity(contact.bodyB, velB + impulse * invMassB);
        }
    }
}

void resolvePositionalCorrections(Engine& engine, std::span<const Contact> contacts,
                                 const SolverConfig& config)
{
    const float slop = (config.slop >= 0.f && std::isfinite(config.slop)) ? config.slop : 0.01f;
    const float percent = clamp01(std::isfinite(config.percent) ? config.percent : 0.4f);
    const float maxCorrection = (config.maxCorrection > 0.f && std::isfinite(config.maxCorrection))
                                    ? config.maxCorrection
                                    : 0.5f;

    for (const Contact& contact : contacts) {
        if (contact.bodyA == contact.bodyB) {
            continue;
        }

        if (!engine.isValidHandle(contact.bodyA) || !engine.isValidHandle(contact.bodyB)) {
            continue;
        }

        const float invMassA = engine.getInverseMass(contact.bodyA);
        const float invMassB = engine.getInverseMass(contact.bodyB);
        const float totalInvMass = invMassA + invMassB;

        if (totalInvMass <= 0.f || !std::isfinite(totalInvMass)) {
            continue;
        }

        const float penetration = contact.penetration;
        if (!std::isfinite(penetration) || penetration <= slop) {
            continue;
        }

        const Vec2 normal = contact.normal;
        if (!std::isfinite(normal.x) || !std::isfinite(normal.y)) {
            continue;
        }

        const float depth = penetration - slop;
        const float rawMagnitude = (depth / totalInvMass) * percent;
        const float correctionMagnitude = std::min(rawMagnitude, maxCorrection);
        if (!std::isfinite(correctionMagnitude) || correctionMagnitude <= 0.f) {
            continue;
        }

        const Vec2 correction = normal * correctionMagnitude;

        if (invMassA > 0.f) {
            const Vec2 posA = engine.getPosition(contact.bodyA);
            engine.setPosition(contact.bodyA, posA - correction * invMassA);
        }
        if (invMassB > 0.f) {
            const Vec2 posB = engine.getPosition(contact.bodyB);
            engine.setPosition(contact.bodyB, posB + correction * invMassB);
        }
    }
}

void solveCollisions(Engine& engine, std::span<const Contact> contacts,
                     const SolverConfig& config)
{
    resolveVelocityImpulses(engine, contacts);
    resolvePositionalCorrections(engine, contacts, config);
}

} // namespace PhysicsEngine
