// WorkerPool.cpp -- Persistent worker thread pool implementation.
//
// See WorkerPool.h for architectural rationale and API contract.

#include "PhysicsEngine/Concurrency/WorkerPool.h"

#include <algorithm>
#include <memory>

namespace PhysicsEngine {

WorkerPool::WorkerPool(uint32_t threadCount)
    : workerCount_(threadCount)
{
    if (workerCount_ > 0) {
        workers_.reserve(workerCount_);
        for (uint32_t i = 0; i < workerCount_; ++i) {
            // Worker threads receive workerIndex in [1, workerCount_]
            // The calling / main thread uses workerIndex = 0
            workers_.emplace_back(&WorkerPool::workerLoop, this, i + 1);
        }
    }
}

WorkerPool::~WorkerPool()
{
    shutdown();
}

bool WorkerPool::isRunning() const noexcept
{
    return !stop_.load(std::memory_order_acquire);
}

void WorkerPool::shutdown() noexcept
{
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        if (stop_.load(std::memory_order_relaxed)) {
            return;
        }
        stop_.store(true, std::memory_order_release);
    }
    queueCv_.notify_all();

    for (auto& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    workers_.clear();
    workerCount_ = 0;
}

void WorkerPool::enqueue(std::function<void(uint32_t)> task)
{
    if (!task) {
        return;
    }

    if (workerCount_ == 0 || stop_.load(std::memory_order_acquire)) {
        task(0);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        if (stop_.load(std::memory_order_relaxed)) {
            task(0);
            return;
        }
        taskQueue_.push(std::move(task));
    }
    queueCv_.notify_one();
}

void WorkerPool::workerLoop(uint32_t workerIndex)
{
    while (true) {
        std::function<void(uint32_t)> task;
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            queueCv_.wait(lock, [this] {
                return stop_.load(std::memory_order_acquire) || !taskQueue_.empty();
            });

            if (stop_.load(std::memory_order_acquire) && taskQueue_.empty()) {
                return;
            }

            task = std::move(taskQueue_.front());
            taskQueue_.pop();
        }

        task(workerIndex);
    }
}

void WorkerPool::parallelFor(size_t count,
                             const std::function<void(size_t start, size_t end, uint32_t workerIndex)>& task,
                             size_t minChunkSize)
{
    if (count == 0 || !task) {
        return;
    }

    if (minChunkSize == 0) {
        minChunkSize = 1;
    }

    // Fast-path: sequential execution on calling thread if zero workers, stopped,
    // or iteration count is within sequential threshold.
    if (workerCount_ == 0 || count <= minChunkSize || stop_.load(std::memory_order_acquire)) {
        task(0, count, 0);
        return;
    }

    struct Chunk {
        size_t start;
        size_t end;
    };

    // Partition work into chunks targeting up to 2x available workers for dynamic load balancing
    const size_t targetChunks = std::max<size_t>(1, static_cast<size_t>(workerCount_ + 1) * 2);
    const size_t chunkSize    = std::max(minChunkSize, (count + targetChunks - 1) / targetChunks);

    std::vector<Chunk> chunks;
    chunks.reserve(targetChunks);
    for (size_t start = 0; start < count; start += chunkSize) {
        const size_t end = std::min(count, start + chunkSize);
        chunks.push_back(Chunk{start, end});
    }

    if (chunks.size() <= 1) {
        task(0, count, 0);
        return;
    }

    struct ParallelState {
        std::vector<Chunk>                            chunks;
        std::function<void(size_t, size_t, uint32_t)> task;
        std::atomic<size_t>                           nextChunk{0};
        std::atomic<size_t>                           completedChunks{0};
        const size_t                                  totalChunks;
        std::atomic<size_t>                           pendingWorkers{0};
        std::atomic<bool>                             hasException{false};
        std::mutex                                    mutex;
        std::condition_variable                       cv;
        std::exception_ptr                            exceptionPtr{nullptr};

        ParallelState(std::vector<Chunk> c,
                      std::function<void(size_t, size_t, uint32_t)> t,
                      size_t workers)
            : chunks(std::move(c))
            , task(std::move(t))
            , totalChunks(chunks.size())
            , pendingWorkers(workers)
        {}

        void execute(uint32_t workerIndex) {
            while (true) {
                const size_t idx = nextChunk.fetch_add(1, std::memory_order_relaxed);
                if (idx >= totalChunks) {
                    break;
                }

                if (!hasException.load(std::memory_order_relaxed)) {
                    try {
                        task(chunks[idx].start, chunks[idx].end, workerIndex);
                    } catch (...) {
                        std::lock_guard<std::mutex> lock(mutex);
                        if (!exceptionPtr) {
                            exceptionPtr = std::current_exception();
                        }
                        hasException.store(true, std::memory_order_release);
                    }
                }

                completedChunks.fetch_add(1, std::memory_order_acq_rel);
            }
        }
    };

    const size_t tasksToEnqueue = std::min<size_t>(chunks.size() - 1, static_cast<size_t>(workerCount_));
    auto state = std::make_shared<ParallelState>(std::move(chunks), task, tasksToEnqueue);

    if (tasksToEnqueue > 0) {
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            for (size_t i = 0; i < tasksToEnqueue; ++i) {
                taskQueue_.push([state](uint32_t wIdx) {
                    state->execute(wIdx);
                    if (state->pendingWorkers.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                        std::lock_guard<std::mutex> lock(state->mutex);
                        state->cv.notify_all();
                    }
                });
            }
        }
        queueCv_.notify_all();
    }

    // Calling thread actively participates using workerIndex = 0
    state->execute(0);

    // Wait until all chunks are finished AND all enqueued tasks have exited
    {
        std::unique_lock<std::mutex> lock(state->mutex);
        state->cv.wait(lock, [&state] {
            return state->completedChunks.load(std::memory_order_acquire) == state->totalChunks &&
                   state->pendingWorkers.load(std::memory_order_acquire) == 0;
        });

        if (state->exceptionPtr) {
            std::rethrow_exception(state->exceptionPtr);
        }
    }
}

} // namespace PhysicsEngine
