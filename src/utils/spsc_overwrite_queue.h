#pragma once
/*
 * spsc_overwrite_queue.hpp
 *
 * Single-producer / single-consumer lock-free ring buffer.
 * - Capacity must be a power of two.
 * - push() never fails; if full, it overwrites the oldest element.
 * - clear_from_producer() resets the queue from the producer side
 *   without losing the first element written afterwards.
 * - clear_from_consumer() is a fast consumer-side purge.
 */

#include <etl/array.h>
#include <etl/atomic.h>
#include <etl/utility.h>

template<typename T, size_t CAP_POW2>
class spsc_overwrite_queue {
    static_assert((CAP_POW2 & (CAP_POW2 - 1)) == 0, "capacity must be 2^k");
    static constexpr size_t MASK = CAP_POW2 - 1;

    etl::array<T, CAP_POW2> buf;
    etl::atomic<size_t> head{0}; // written only by producer
    etl::atomic<size_t> tail{0};

public:
    template<typename U>
    void push(U&& v) {
        auto h = head.load(etl::memory_order_relaxed);

        // Check overflow and release space if needed
        while (true) {
            auto current_tail = tail.load(etl::memory_order_relaxed);

            // If the queue is not full, we can write
            if (h + 1 - current_tail <= CAP_POW2) { break; }

            // Ensure that the tail is not changed by the consumer
            // while we are trying to write. If it is, we will retry.
            if (tail.compare_exchange_weak(current_tail, current_tail + 1,
                etl::memory_order_relaxed, etl::memory_order_relaxed)) {
                break;
            }
        }

        // Now we a guaranteed to have free space, it is safe to write
        // and increment head.
        buf[h & MASK] = etl::forward<U>(v);
        head.store(h + 1, etl::memory_order_release);
    }

    void clear_from_producer() {
        auto h = head.load(etl::memory_order_relaxed);
        auto expected = tail.load(etl::memory_order_relaxed);

        // retry until tail == h
        while (!tail.compare_exchange_weak(expected, h,
            etl::memory_order_release, etl::memory_order_relaxed)) {}
    }

    bool pop(T& out) {
        auto t = tail.load(etl::memory_order_relaxed);
        while (true) {
            auto h = head.load(etl::memory_order_acquire);

            // fail on empty queue
            if (t == h) { return false; }

            T temp = buf[t & MASK];
            // advance tail only if it has not changed
            if (tail.compare_exchange_weak(t, t + 1,
                etl::memory_order_release, etl::memory_order_relaxed)) {
                out = temp;
                return true;
            }
        }
    }

    void clear_from_consumer() {
        auto t = tail.load(etl::memory_order_relaxed);
        auto h = head.load(etl::memory_order_acquire);

        while (true) {
            auto dist = h - t;

            // If the queue is empty OR producer is writing too much of new data
            // with high pressure - nothing to clear.
            if (dist == 0 || dist > CAP_POW2) { return;}

            if (tail.compare_exchange_weak(t, h,
                etl::memory_order_release, etl::memory_order_relaxed)) {
                return;
            }
        }
    }

    bool empty() const {
        // false "empty" not allowed, false "not empty" - possible.
        // use `.pop()` status for stronger guarantee.
        auto t = tail.load(etl::memory_order_relaxed);
        auto h = head.load(etl::memory_order_acquire);
        return t == h;
    }
};
