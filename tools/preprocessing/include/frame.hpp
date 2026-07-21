#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <queue>
#include <vector>

namespace preprocessing {

// One already-undistorted, grayscale RGB camera frame, ready for AprilTag
// detection. timestamp_ns comes straight from the VRS record -- no more
// separate mp4_to_vrs_time_ns.json lookup table needed, since the SDK
// hands us the real capture timestamp per frame.
struct DecodedFrame {
    size_t frame_index;
    int64_t timestamp_ns;
    size_t width;
    size_t height;
    // Tightly packed: row length == width, no per-row padding.
    std::vector<uint8_t> gray_data;
};

// A small blocking bounded queue -- the hand-rolled equivalent of a
// crossbeam bounded channel. push() blocks while full; pop() blocks while
// empty. Once close() has been called, pop() drains whatever is left and
// then starts returning false.
template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(size_t capacity) : capacity_(capacity) {}

    bool push(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_full_.wait(lock, [this] { return queue_.size() < capacity_ || closed_; });
        if (closed_) {
            return false;
        }
        queue_.push(std::move(item));
        lock.unlock();
        not_empty_.notify_one();
        return true;
    }

    bool pop(T& out) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [this] { return !queue_.empty() || closed_; });
        if (queue_.empty()) {
            return false;
        }
        out = std::move(queue_.front());
        queue_.pop();
        lock.unlock();
        not_full_.notify_one();
        return true;
    }

    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        not_empty_.notify_all();
        not_full_.notify_all();
    }

private:
    std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    std::queue<T> queue_;
    size_t capacity_;
    bool closed_ = false;
};

} // namespace preprocessing
