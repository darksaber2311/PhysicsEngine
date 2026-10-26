# PhysicsEngine2D

A high-performance, data-oriented 2D rigid-body physics engine and interactive sandbox implemented in **Modern C++20** from scratch.

Designed with clean module decoupling: the core physics and mathematical subsystems compile into a **completely headless static library** (`PhysicsEngine`) with zero graphics dependencies, while the interactive application (`PhysicsEngineApp`) provides real-time visualization and controls via GLFW, OpenGL 3.3 Core profile, and a custom immediate-mode 2D GUI.

---

## Architecture & Subsystem Boundaries

The project is structured into three clean layers:

```
PhysicsEngine2D
├── PhysicsEngine (Headless Static Library)
│   ├── Math
│   │   └── Vec2                # Vector math: dot, perp, length, normalization, distance
│   ├── Physics
│   │   ├── Engine              # SoA body storage, generation-checked BodyHandles, capacity checks
│   │   ├── Integration         # Symplectic (semi-implicit) Euler integrator with force clearing
│   │   ├── Collision           # Narrowphase: Circle-Circle, AABB-AABB, Circle-AABB, BruteForce
│   │   ├── SpatialGrid         # Broadphase: Uniform grid, duplicate suppression, oversized fallback
│   │   ├── CollisionSolver     # Sequential impulse solver and Baumgarte positional correction
│   │   └── SimulationController# Fixed-timestep accumulator (fixedDt = 1/120s), pause/single-step
│   └── Concurrency
│       └── WorkerPool          # Persistent C++20 thread pool with lock-free atomic chunk dispatch
├── PhysicsEngineApp (Interactive Visualization & Sandbox)
│   ├── Camera2D                # 2D orthographic camera with aspect preservation, pan, and zoom
│   ├── Shader                  # RAII OpenGL 3.3 Core shader program compilation & uniform management
│   ├── Renderer2D              # Batched GPU drawing: dynamic/static shapes, contacts, grids, HUD
│   ├── UI (Immediate-Mode GUI) # Self-contained UI: buttons, toggles, steppers, scrollable panel
│   └── main.cpp                # Application loop, input disambiguation, and runtime controls
├── tests                       # 218 Catch2 unit and regression tests (completely headless)
└── benchmarks                  # Headless benchmark suite measuring broad-phase scaling (N = 100..5000)
```

---

## Build Requirements

- **C++ Compiler**: Modern C++20 compliant compiler:
  - GCC 12+ (tested with GCC 15.2.0)
  - Clang 15+
  - MSVC 2022+ (v143 toolset)
- **CMake**: Version 3.25 or newer
- **Ninja** or platform build system
- **OpenGL**: Desktop OpenGL 3.3 Core support

All third-party libraries (GLFW 3.4, Catch2 v3.7.1, GLAD2) are automatically managed via CMake `FetchContent`.

---

## Build Instructions

### Debug Build

```bash
# Configure debug preset
cmake -B build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug

# Compile application, tests, and benchmarks
cmake --build build/debug

# Run Catch2 unit test suite (218 tests)
ctest --test-dir build/debug --output-on-failure

# Launch the interactive sandbox application
./build/debug/bin/PhysicsEngineApp.exe

# Headless verification run (e.g. 180 frames)
./build/debug/bin/PhysicsEngineApp.exe --frames 180
```

### Release Build

```bash
# Configure release preset
cmake -B build/release -G Ninja -DCMAKE_BUILD_TYPE=Release

# Compile release binaries
cmake --build build/release --target PhysicsEngineTests PhysicsEngineBenchmarks

# Run headless test suite in release mode
ctest --test-dir build/release --output-on-failure

# Run broad-phase benchmark harness
./build/release/bin/PhysicsEngineBenchmarks.exe
```

---

## Interactive Sandbox Controls

The application turns the physics simulation into an interactive laboratory with zero external GUI dependencies.

### Mouse Interactions
- **Left-Click (Viewport)**: Spawns the currently configured body (Circle or AABB) at the exact cursor world position.
- **Left-Click (UI Panel)**: Interacts with UI buttons, toggles, steppers, and choices. Pointer capture guarantees UI clicks **never** spawn bodies or pan the camera.
- **Drag (Right, Middle, or Left > 6px)**: Smoothly pans the camera across world space. Releasing a pan gesture **never** spawns a body.
- **Mouse Wheel**:
  - Over Viewport: Zooms the camera in/out toward the cursor.
  - Over UI Panel: Smoothly scrolls the UI control panel, ensuring all widgets remain accessible on small window heights.

### Keyboard Shortcuts
| Key | Action |
|---|---|
| `Space` | Pause / Resume simulation |
| `S` | Advance exactly **one** fixed physics step ($fixedDt = 1/120\text{ s}$) while paused |
| `R` | Reset scene to deterministic initial layout, bodies, and gravity |
| `C` | Toggle contact point crosshairs and normal indicator arrows |
| `G` | Toggle world coordinate grid and axes |
| `B` | Toggle spatial grid cell boundaries |
| `Escape` | Close application |

*Note*: Keyboard shortcuts are inhibited when the mouse cursor is over the UI panel to prevent accidental triggers while configuring parameters.

### UI Control Panel

1. **Simulation Controls**:
   - **Pause / Resume**: Toggles continuous physics progression.
   - **Single-Step [S]**: Advances exactly 1 fixed timestep without altering or consuming the real-time accumulator, leaving the simulation paused.
   - **Reset Scene [R]**: Restores initial scene geometry, resets the accumulator, clears contact buffers, and updates status.
2. **Gravity Controls**:
   - **X & Y Steppers**: Live steppers adjusting gravitational acceleration ($[-50, +50]\text{ m/s}^2$).
   - **Presets**: One-click presets for **Earth** ($0, -9.81$), **Moon** ($0, -1.62$), **Zero-G** ($0, 0$), and **Inverted** ($0, +9.81$).
3. **Broad-Phase Selector**:
   - **BruteForce**: $O(N^2)$ reference pairwise detector.
   - **Grid (Seq)**: Uniform spatial grid sequential detector (engine default).
   - **Grid (Par)**: Multithreaded spatial grid broad-phase backed by a persistent `WorkerPool`.
4. **Spawn Configuration**:
   - **Shape**: Circle or AABB Box.
   - **Type**: Dynamic (positive mass) or Static (infinite mass / $invMass = 0$).
   - **Geometry**: Radius or Half-Width / Half-Height ($0.2\text{ m}$ to $5.0\text{ m}$).
   - **Physics Parameters**: Mass ($0.1\text{ kg}$ to $100.0\text{ kg}$), Restitution ($[0.0, 1.0]$), and Initial Launch Velocity ($V_x, V_y$).
5. **Debug Visualization**:
   - Show Contacts, Show World Grid, Show Grid Cell Boundaries.
6. **Real-time Feedback Banner**:
   - Provides visible confirmation of body spawns and warns when reaching the body capacity limit ($1024$ bodies).

---

## Broad-Phase Performance Benchmarks

The benchmark suite (`PhysicsEngineBenchmarks`) executes deterministic comparative evaluations across identical scenes with constant density ($\rho \approx 0.05\text{ bodies/m}^2$, 50% Circles, 50% AABBs, 10% static):

### Benchmark Methodology & Timed Region

- **Equivalent Workload**: All three algorithms (`BruteForce`, `SpatialGrid` sequential, and `SpatialGridParallel`) perform the exact same end-to-end task: taking an `Engine` instance, detecting candidate pairs, confirming geometric narrow-phase collisions (`testCollisionSlots`), and populating a `std::vector<Contact>` output buffer.
- **Timed Region Scope**: Timing measures the complete detection pipeline:
  1. Grid construction, conservative AABB computation, and cell hashing (or pairwise scan in brute force).
  2. Multi-cell scan, candidate pair generation, and duplicate suppression.
  3. Narrow-phase geometric intersection checks (Circle–Circle, AABB–AABB, Circle–AABB), computing contact points, penetration depths, and normal vectors.
  4. Merging confirmed contacts into the output buffer.
- **Separation of Setup**: Scene generation, memory pre-allocation, thread pool construction, and warm-up passes occur strictly outside the timed loop.
- **Concurrency Architecture**: The multithreaded broad-phase uses a **dynamic atomic chunk-dispatching scheduler** (via atomic monotonic counter fetch-and-add) with calling-thread participation, not deque-based work-stealing. Per-worker contact buffers guarantee lock-free chunk processing and deterministic contact merging.
- **Bit-for-Bit Determinism**: The benchmark verifies that `pairsBF == pairsSeq == pairsPar2 == pairsPar4 == pairsPar8` across all iterations and thread configurations.

### Representative Release Results (GCC 15.2.0, Windows x86_64)

| Bodies | Domain | BF Candidate Pairs | Grid Candidate Pairs | Contacts | Brute-Force (Median) | Spatial Grid Seq (Median) | Parallel 2T (Median) | Parallel 4T (Median) | Parallel 8T (Median) | Grid vs BF Speedup | Par 8T vs Seq Speedup |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **100** | $44.7\text{ m}$ | 4,922 | 72 | 15 | $0.065\text{ ms}$ | **$0.005\text{ ms}$** | $0.015\text{ ms}$ | $0.017\text{ ms}$ | $0.017\text{ ms}$ | **13.0x** | $0.29\text{x}^\dagger$ |
| **500** | $100.0\text{ m}$ | 124,222 | 417 | 75 | $1.702\text{ ms}$ | **$0.028\text{ ms}$** | $0.064\text{ ms}$ | $0.071\text{ ms}$ | $0.098\text{ ms}$ | **60.8x** | $0.29\text{x}^\dagger$ |
| **1,000** | $141.4\text{ m}$ | 496,725 | 791 | 143 | $7.034\text{ ms}$ | **$0.103\text{ ms}$** | $0.127\text{ ms}$ | $0.147\text{ ms}$ | $0.165\text{ ms}$ | **68.3x** | $0.63\text{x}^\dagger$ |
| **2,500** | $223.6\text{ m}$ | 3,099,440 | 1,948 | 383 | $47.523\text{ ms}$ | $0.350\text{ ms}$ | **$0.338\text{ ms}$** | $0.348\text{ ms}$ | $0.357\text{ ms}$ | **135.8x** | $0.98\text{x}$ |
| **5,000** | $316.2\text{ m}$ | 12,394,669 | 3,805 | 753 | $191.036\text{ ms}$ | $0.810\text{ ms}$ | $0.855\text{ ms}$ | $0.793\text{ ms}$ | **$0.699\text{ ms}$** | **235.8x** | **1.16x** |

$^\dagger$**Amdahl's Law & Concurrency Overhead**: For small scenes ($N \le 1000$), work chunk dispatch latency and synchronization exceed the raw compute work of narrow-phase pair tests. The engine defaults to **Sequential Spatial Grid** and uses a configurable sequential threshold (`sequentialThreshold = 256`) to ensure optimal throughput across all scene scales.

**Bit-for-Bit Determinism**: The benchmark verifies that `pairsBF == pairsSeq == pairsPar2 == pairsPar4 == pairsPar8` across all iterations and thread configurations.

---

## Supported Collision Geometry & Known Limitations

### Supported Geometry
- **Circle**: Defined by center position and radius.
- **AABB**: Axis-aligned bounding box defined by center position and half-extents.
- **Narrow-Phase Tests**: Circle–Circle, AABB–AABB, and Circle–AABB. All collision routines produce consistent separation normals (pointing strictly from `bodyA` to `bodyB`) and positive penetration depths.

### Current Physics Limitations
- **Discrete Collision Detection**: The engine uses discrete collision detection. Fast-moving objects (relative velocity $\times\ fixedDt > \text{diameter}$) can exhibit tunnelling through thin static barriers. Continuous Collision Detection (CCD) is deferred to future work.
- **Rotational Dynamics**: Bodies possess linear translational velocity ($V_x, V_y$) only. Angular velocity, torque, moment of inertia, and friction impulses are not yet implemented.
- **Static Stacking Convergence**: The sequential impulse solver uses Baumgarte positional correction. High stacks of dynamic bodies ($> 8$ layers) can exhibit minor settling drift or elastic bounce without contact manifold warm-starting or shock propagation.

---

## License

This project is licensed under the MIT License — see the [`LICENSE`](LICENSE) file for details.
