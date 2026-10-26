// tests/test_worker_pool.cpp -- Unit tests for PhysicsEngine::WorkerPool.

#include "PhysicsEngine/Concurrency/WorkerPool.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <atomic>
#include <chrono>
#include <numeric>
#include <stdexcept>
#include <vector>

using namespace PhysicsEngine;

TEST_CASE("WorkerPool: constructor respects configurable worker count", "[worker_pool]")
{
    // Zero workers (sequential mode)
    {
        WorkerPool pool0(0);
        REQUIRE(pool0.workerCount() == 0);
        REQUIRE(pool0.isRunning());
    }

    // 1 worker
    {
        WorkerPool pool1(1);
        REQUIRE(pool1.workerCount() == 1);
        REQUIRE(pool1.isRunning());
    }

    // 4 workers
    {
        WorkerPool pool4(4);
        REQUIRE(pool4.workerCount() == 4);
        REQUIRE(pool4.isRunning());
    }
}

TEST_CASE("WorkerPool: zero workers executes tasks synchronously without deadlock", "[worker_pool]")
{
    WorkerPool pool(0);

    std::vector<int> data(100, 0);
    pool.parallelFor(data.size(), [&](size_t start, size_t end, uint32_t workerIndex) {
        REQUIRE(workerIndex == 0);
        for (size_t i = start; i < end; ++i) {
            data[i] = static_cast<int>(i * 2);
        }
    });

    for (size_t i = 0; i < data.size(); ++i) {
        REQUIRE(data[i] == static_cast<int>(i * 2));
    }
}

TEST_CASE("WorkerPool: single worker executes tasks completely", "[worker_pool]")
{
    WorkerPool pool(1);

    std::vector<int> data(250, 0);
    pool.parallelFor(data.size(), [&](size_t start, size_t end, uint32_t /*workerIndex*/) {
        for (size_t i = start; i < end; ++i) {
            data[i] = 1;
        }
    });

    for (int val : data) {
        REQUIRE(val == 1);
    }
}

TEST_CASE("WorkerPool: empty tasks and boundary parameters", "[worker_pool]")
{
    WorkerPool pool(4);

    // Count = 0 returns immediately
    bool invoked = false;
    pool.parallelFor(0, [&](size_t, size_t, uint32_t) {
        invoked = true;
    });
    REQUIRE_FALSE(invoked);

    // Null task callback is handled safely
    pool.parallelFor(10, nullptr);

    // minChunkSize = 0 is sanitized to 1
    std::atomic<int> sum{0};
    pool.parallelFor(50, [&](size_t start, size_t end, uint32_t) {
        for (size_t i = start; i < end; ++i) {
            sum.fetch_add(1, std::memory_order_relaxed);
        }
    }, 0);
    REQUIRE(sum.load() == 50);
}

TEST_CASE("WorkerPool: parallelFor synchronization guarantees full completion", "[worker_pool]")
{
    WorkerPool pool(4);

    constexpr size_t N = 10000;
    std::vector<uint64_t> values(N, 0);

    pool.parallelFor(N, [&](size_t start, size_t end, uint32_t) {
        for (size_t i = start; i < end; ++i) {
            values[i] = i * 3 + 7;
        }
    }, 64);

    // After parallelFor returns, all values must be computed
    for (size_t i = 0; i < N; ++i) {
        REQUIRE(values[i] == (i * 3 + 7));
    }
}

TEST_CASE("WorkerPool: exception safety captures and rethrows on calling thread", "[worker_pool]")
{
    WorkerPool pool(4);

    constexpr size_t N = 500;
    std::vector<int> touched(N, 0);

    REQUIRE_THROWS_AS(
        pool.parallelFor(N, [&](size_t start, size_t end, uint32_t) {
            for (size_t i = start; i < end; ++i) {
                touched[i] = 1;
                if (i == 250) {
                    throw std::runtime_error("Simulated worker error at index 250");
                }
            }
        }, 32),
        std::runtime_error
    );

    // Pool remains completely functional after an exception
    REQUIRE(pool.isRunning());

    std::atomic<int> counter{0};
    pool.parallelFor(100, [&](size_t start, size_t end, uint32_t) {
        for (size_t i = start; i < end; ++i) {
            counter.fetch_add(1, std::memory_order_relaxed);
        }
    });
    REQUIRE(counter.load() == 100);
}

TEST_CASE("WorkerPool: shutdown is clean and idempotent", "[worker_pool]")
{
    WorkerPool pool(4);
    REQUIRE(pool.isRunning());

    // Explicit shutdown
    pool.shutdown();
    REQUIRE_FALSE(pool.isRunning());

    // Repeated shutdown is safe and idempotent
    pool.shutdown();
    pool.shutdown();
    REQUIRE_FALSE(pool.isRunning());

    // Subsequent parallelFor runs sequentially on calling thread without hanging
    int executed = 0;
    pool.parallelFor(10, [&](size_t start, size_t end, uint32_t) {
        for (size_t i = start; i < end; ++i) {
            ++executed;
        }
    });
    REQUIRE(executed == 10);
}

TEST_CASE("WorkerPool: repeated submissions stress test", "[worker_pool]")
{
    WorkerPool pool(4);

    constexpr int kIterations = 100;
    constexpr size_t kSize = 1000;
    std::vector<int> buffer(kSize, 0);

    for (int iter = 0; iter < kIterations; ++iter) {
        pool.parallelFor(kSize, [&](size_t start, size_t end, uint32_t) {
            for (size_t i = start; i < end; ++i) {
                buffer[i] = iter;
            }
        }, 50);

        REQUIRE(buffer[0] == iter);
        REQUIRE(buffer[kSize - 1] == iter);
    }
}

TEST_CASE("WorkerPool: submit returns valid future with result", "[worker_pool]")
{
    WorkerPool pool(2);

    auto fut1 = pool.submit([] {
        return 42;
    });

    auto fut2 = pool.submit([](int a, int b) {
        return a + b;
    }, 10, 25);

    REQUIRE(fut1.get() == 42);
    REQUIRE(fut2.get() == 35);
}
