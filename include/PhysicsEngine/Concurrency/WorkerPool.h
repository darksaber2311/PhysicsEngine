#pragma once

// WorkerPool.h -- Persistent worker thread pool for PhysicsEngine2D.
//
// Key Responsibilities & Invariants:
//   1. Persistent Threads:
//      Worker threads are created once during pool construction and persist
//      until pool shutdown or destruction. No thread creation or destruction
//      occurs during physics simulation steps.
//   2. Zero-Worker Mode (Sequential Fallback):
//      If workerCount == 0, the pool operates in sequential mode: all submitted
//      tasks and parallel-for iterations execute synchronously on the calling thread.
//      Guaranteed never to deadlock.
//   3. Single-Worker Mode:
//      Fully supported without deadlocks or priority inversions.
//   4. Exception Safety:
//      If any chunk in a parallelFor loop throws an exception, the first thrown
//      exception is safely captured (std::exception_ptr), remaining tasks complete
//      gracefully, and the exception is rethrown on the calling thread.
//   5. Calling Thread Participation:
//      In parallelFor, the calling thread actively participates in executing chunks
//      via dynamic atomic chunk distribution (monotonic fetch_add counter), eliminating idle blocking.
//   6. Clean Shutdown:
//      Destructor and shutdown() ensure all worker threads are joined cleanly.
//      shutdown() is idempotent and safe to invoke multiple times.

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace PhysicsEngine {

class WorkerPool {
public:
    /// Constructs a persistent worker pool with the specified number of background threads.
    /// \param threadCount  Number of background threads. If 0, operations execute
    ///                     synchronously on the calling thread.
    explicit WorkerPool(uint32_t threadCount = std::thread::hardware_concurrency());

    /// Destructor cleanly joins all worker threads.
    ~WorkerPool();

    WorkerPool(const WorkerPool&) = delete;
    WorkerPool& operator=(const WorkerPool&) = delete;
    WorkerPool(WorkerPool&&) = delete;
    WorkerPool& operator=(WorkerPool&&) = delete;

    /// Returns the number of background worker threads in the pool.
    /// 0 indicates sequential execution on the calling thread.
    [[nodiscard]] uint32_t workerCount() const noexcept { return workerCount_; }

    /// Returns true if the pool is running and accepting tasks.
    [[nodiscard]] bool isRunning() const noexcept;

    /// Shuts down the pool, finishes any in-flight tasks, and joins all worker threads.
    /// Safe to call multiple times or before destruction.
    void shutdown() noexcept;

    /// Submits a callable to be executed asynchronously, returning a std::future.
    /// If workerCount() == 0 or the pool is stopped, the task executes immediately
    /// on the calling thread.
    template <typename F, typename... Args>
    auto submit(F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>>
    {
        using ReturnType = std::invoke_result_t<F, Args...>;

        auto task = std::make_shared<std::packaged_task<ReturnType()>>(
            [func = std::forward<F>(f), ...captureArgs = std::forward<Args>(args)]() mutable {
                return std::invoke(std::move(func), std::move(captureArgs)...);
            }
        );

        std::future<ReturnType> fut = task->get_future();

        if (workerCount_ == 0 || stop_.load(std::memory_order_acquire)) {
            (*task)();
            return fut;
        }

        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            if (stop_.load(std::memory_order_relaxed)) {
                (*task)();
                return fut;
            }
            taskQueue_.push([task](uint32_t /*workerIndex*/) {
                (*task)();
            });
        }
        queueCv_.notify_one();
        return fut;
    }

    /// Executes a parallel-for loop over [0, count) partitioned across available workers.
    ///
    /// - If count == 0: returns immediately without doing any work.
    /// - If workerCount() == 0 or count <= minChunkSize: executes task(0, count, 0)
    ///   synchronously on the calling thread.
    /// - If any task throws an exception, the exception is captured and rethrown
    ///   on the calling thread after all tasks conclude.
    /// - The calling thread actively participates in executing chunks (workerIndex = 0).
    /// - Background workers receive unique workerIndex in [1, workerCount()].
    ///
    /// \param count         Total number of elements in iteration space [0, count).
    /// \param task          Callback with signature: void(size_t start, size_t end, uint32_t workerIndex).
    /// \param minChunkSize  Minimum elements per chunk.
    void parallelFor(size_t count,
                     const std::function<void(size_t start, size_t end, uint32_t workerIndex)>& task,
                     size_t minChunkSize = 1);

    /// Low-level task enqueue.
    void enqueue(std::function<void(uint32_t workerIndex)> task);

private:
    void workerLoop(uint32_t workerIndex);

    uint32_t                                  workerCount_ = 0;
    std::vector<std::thread>                  workers_;
    std::queue<std::function<void(uint32_t)>> taskQueue_;
    mutable std::mutex                        queueMutex_;
    std::condition_variable                   queueCv_;
    std::atomic<bool>                         stop_{false};
};

} // namespace PhysicsEngine
