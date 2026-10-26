# Collision Detection

> Updated: Milestone 1 (stub — detail added in Milestone 4)

## Shapes Supported

| Shape | Parameters | Collision type |
|-------|-----------|----------------|
| Circle | centre, radius | Circle–Circle |
| AABB   | min-corner, max-corner | AABB–AABB |

Cross-shape pairs (Circle vs AABB) may be added in a later milestone.

## Pipeline

```
Broad Phase  →  Candidate pairs  →  Narrow Phase  →  Contacts  →  Solver
```

Broad phase reduces the N² pair problem to a smaller candidate set using
spatial acceleration. Narrow phase applies exact geometry tests. The solver
consumes the contact manifold.

## Broad Phase Options

| Strategy | Complexity | Notes |
|----------|-----------|-------|
| Brute Force | O(N²) | Correctness baseline |
| Uniform Grid | O(N) amortised | Cell size must be tuned |
| Quadtree | O(N log N) avg | Degrades with large / uneven objects |

## Contact Representation

```cpp
struct Contact {
    BodyHandle bodyA;
    BodyHandle bodyB;
    Vec2       normal;    // Points from A toward B
    float      depth;     // Penetration depth (positive = overlapping)
    Vec2       point;     // Contact point (world space)
};
```
