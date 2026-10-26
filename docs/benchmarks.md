# Benchmarks

> Updated: Milestone 1 (stub — measurements added in Milestone 8)

## Methodology

- All benchmarks use **deterministic seeded** scene generation.
- Warm-up runs are discarded before timing.
- Each benchmark reports **median** and **95th percentile** wall-clock time.
- Hardware, compiler, build flags, and timestep are recorded alongside results.
- No results are invented; every number reflects an actual measurement.

## Planned Comparisons

| Experiment | What it measures |
|-----------|-----------------|
| SoA vs AoS | Cache locality impact on integration |
| Serial vs parallel integration | Thread-pool speedup |
| Brute force vs spatial grid | Broad-phase scalability |
| Spatial grid vs quadtree | Broad-phase at different distributions |
| Worker count scaling | Parallel efficiency |

## Running Benchmarks

```bash
cmake --preset release
cmake --build build/release
./build/release/bin/PhysicsEngineBenchmarks
```

Results are logged to stdout in a machine-parseable format (TBD in Milestone 8).

## Hardware Reference

To be filled in when benchmarks are implemented.

| Field | Value |
|-------|-------|
| CPU | TBD |
| RAM | TBD |
| OS  | TBD |
| Compiler | TBD |
| Build flags | RelWithDebInfo |
