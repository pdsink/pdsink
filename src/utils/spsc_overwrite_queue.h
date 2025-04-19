#pragma once
/*
 * spsc_overwrite_queue.hpp
 *
 * Single‑producer / single‑consumer lock‑free ring buffer based on
 * odd/even head counter.
 *   • Capacity (CAP_POW2) must be a power of two.
 *   • push() never blocks; on overflow the oldest element is discarded.
 *   • head_with_flag is even when ready, odd when writing
 */

#include <etl/array.h>
#include <etl/atomic.h>
#include <etl/utility.h>

template<typename T, size_t CAP_POW2>
class spsc_overwrite_queue {
    static_assert((CAP_POW2 & (CAP_POW2 - 1)) == 0, "Capacity must be 2^k");
    static_assert(CAP_POW2 <= 0x7FFFFFFFu, "Capacity must fit signed-diff logic");

    static constexpr size_t CAP = CAP_POW2;
    static constexpr size_t MASK = CAP - 1;

    union head_fields_t {
        uint32_t raw;
        struct {
            uint32_t writing : 1;
            uint32_t head : 31;
        };
    };

    etl::array<T, CAP> buf;
    etl::atomic<uint32_t> head_fields{0};  // even = ready, odd = writing
    uint32_t tail{0};

    // for reset functionality
    etl::atomic<uint32_t> reset_pos{0};
    etl::atomic<uint32_t> reset_ver{0};
    uint32_t local_ver{0};

    bool check_reset() {
        auto ver = reset_ver.load(etl::memory_order_acquire);
        if (ver != local_ver) {
            tail = reset_pos.load(etl::memory_order_relaxed);
            local_ver = ver;
            return true;
        }
        return false;
    }

    uint32_t get_adjusted_tail(head_fields_t hf) {
        uint32_t max_distance = hf.writing ? CAP - 1 : CAP;

        if (hf.head - tail > max_distance) {
            return (hf.head - max_distance) & 0x7FFFFFFF;
        }
        return tail;
    }

public:

    template<typename U>
    void push(U&& v) noexcept {
        // make odd
        head_fields_t hf{head_fields.fetch_add(1, etl::memory_order_release)};
        // update data
        buf[hf.head & MASK] = etl::forward<U>(v);
        // make even
        head_fields.fetch_add(1, etl::memory_order_release);
    }

    ETL_NODISCARD bool pop(T& out) noexcept {
        while (true) {
            check_reset();

            head_fields_t hf{head_fields.load(etl::memory_order_acquire)};
            auto t = get_adjusted_tail(hf);

            if (t == hf.head) { return false; }  // no data

            T tmp = buf[t & MASK]; // read data

            // Check data not overwritten during read (tail not moved)
            hf.raw = head_fields.load(etl::memory_order_acquire);
            if (get_adjusted_tail(hf) != t) { continue; }

            // Check data not discarded by new reset
            if (check_reset()) { continue; }

            out = tmp;
            tail = (t + 1) & 0x7FFFFFFF;
            return true;
        }
    }

    void clear_from_producer() {
        head_fields_t hf{head_fields.load(etl::memory_order_relaxed)};

        // Store reset position; publication to the consumer is ensured by the
        // release on reset_ver below.
        reset_pos.store(hf.head, etl::memory_order_relaxed);
        // release: makes reset_pos visible before the version bump is observed
        reset_ver.fetch_add(1, etl::memory_order_release);
    }

    void clear_from_consumer() {
        head_fields_t hf{head_fields.load(etl::memory_order_acquire)};
        tail = hf.head;
    }

    bool empty() {
        check_reset();

        head_fields_t hf{head_fields.load(etl::memory_order_acquire)};
        return hf.head == tail;
    }
};
