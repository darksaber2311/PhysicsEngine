# Architecture

> Updated: Milestone 1

## Subsystem Boundaries

```
PhysicsEngine2D
├── Math              — Vec2 and related helpers (header-only)
├── Physics           — Engine, Body, integration, solver
│   ├── Integration   — Semi-implicit Euler over SoA arrays
│   ├── Collision     — Narrow-phase detection (circle, AABB)
│   └── Solver        — Impulse-based resolution
├── Spatial           — Broad-phase acceleration structures
│   ├── BruteForce    — O(N²) reference path
│   ├── SpatialGrid   — Uniform grid with cell deduplication
│   └── Quadtree      — Recursive spatial subdivision
├── Parallel          — Thread pool, job system
└── Rendering         — GLFW/OpenGL; isolated from simulation state
    └── DebugDraw     — Contacts, normals, partitions, HUD
```

## Key Invariants

1. `PhysicsEngine` (the static library) has **no dependency** on GLFW, OpenGL, or GLAD.
2. Tests and benchmarks link only the library — no display required.
3. The rendering layer reads simulation state through a stable public API; it never modifies physics data.
4. Entity handles carry a generation counter (added in Milestone 2) to detect use-after-free.

## Data Layout

Bodies are stored in Structure of Arrays (SoA) layout inside `Engine`. The motivation is cache locality during the integration loop, which reads position and velocity from every body sequentially.

```
Engine:
  positions[]       Vec2  — world-space body centres
  velocities[]      Vec2
  accelerations[]   Vec2  — summed forces / inverseMass per step
  inverseMasses[]   float — 0 = static (infinite mass)
  restitutions[]    float — [0, 1]
  shapeTypes[]      enum  — Circle | AABB
  shapeParams[]     float[2] — radius, OR half-extents (w, h)
  generations[]     uint32 — incremented on body removal
  active[]          bool  — slot in use
```

## Memory Policy

- Capacity is preallocated at construction via `std::vector::reserve`.
- The hot simulation loop must not trigger heap allocation under normal operation.
- Body removal uses **swap-and-pop**: the removed slot is filled with the last active body, and the generation counter of the vacated slot is incremented. External handles validated against both index and generation.

## Threading Model (Milestone 7)

- A persistent `ThreadPool` is created once and lives for the duration of the simulation.
- Body integration is parallelised over disjoint index ranges (no shared writes).
- Collision resolution remains single-threaded initially.
- Parallel collision resolution requires an explicit safe strategy (contact-graph colouring or disjoint batches) before it is introduced.
