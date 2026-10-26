# Parallelism

> Updated: Milestone 1 (stub — implementation in Milestone 7)

## ThreadPool Design

A single `ThreadPool` instance is created once at engine startup and destroyed at shutdown.
Worker threads are **never** created or destroyed per frame.

### Job Model

- Work is submitted as ranges `[begin, end)` over body arrays.
- Each worker processes a contiguous chunk — no false sharing if chunk size ≥ one cache line.
- Completion is signalled via a `std::barrier` or `std::latch` (to be measured).

## Safety Rules

1. Body creation and destruction happen **outside** parallel update ranges.
2. Integration: each body's slot is written by exactly one worker.
3. Collision solver: single-threaded initially. Parallelisation requires
   contact-graph colouring or disjoint batches.
4. No worker may hold a lock during the integration loop — only barrier sync.

## Race Detection

Use `-fsanitize=thread` (TSan) on GCC/Clang builds to verify freedom from
data races before claiming the implementation is correct.
