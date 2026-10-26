# Physics

> Updated: Milestone 1 (stub — integration detail added in Milestone 3)

## Integration

We use **semi-implicit Euler** (also called symplectic Euler):

```
v(t+dt) = v(t) + a(t) * dt      ← velocity updated first
x(t+dt) = x(t) + v(t+dt) * dt  ← position uses the NEW velocity
```

The crucial difference from explicit Euler is that position is advanced using
the *updated* velocity. This makes the method **symplectic**: it conserves a
modified energy over long integrations, giving more stable oscillatory motion
(e.g. pendulums, springs) than explicit Euler with the same step size.

For our use-case (rigid bodies with impulse resolution) it is the standard
industry choice: Box2D and Chipmunk use the same integrator.

## Fixed Timestep

The simulation uses a fixed `dt` (e.g. 1/120 s). A frame-time accumulator
tracks leftover real time between rendered frames:

```
accumulator += frame_time
while (accumulator >= dt):
    step(dt)
    accumulator -= dt
```

**Max steps per frame** is capped (e.g. 8) to prevent the spiral of death
(when physics falls behind real time and tries to catch up with more steps,
which makes it fall further behind).

## Impulse Solver

See `docs/physics.md` (Milestone 5 update) for the derivation of the normal
impulse formula.
