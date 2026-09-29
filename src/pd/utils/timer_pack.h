#pragma once

#include "atomic_bits.h"

#include <etl/array.h>
#include <etl/atomic.h>
#include <etl/optional.h>

namespace pd {

template<size_t TIMER_COUNT>
class TimerPack {
public:
    explicit TimerPack() {
        active.clear_all();
        disabled.set_all();
        timers_changed.store(false);
    }

    void start(int timer_id, uint32_t deadline) {
        active.set(timer_id);
        disabled.clear(timer_id);
        expire_at[timer_id] = deadline;
        timers_changed.store(true);
    }

    void stop(int timer_id) {
        active.clear(timer_id);
        disabled.set(timer_id);
        timers_changed.store(true);
    }

    void stop_range(int first, int last) {
        for (int i = first; i <= last; i++) { stop(i); }
        timers_changed.store(true);
    }

    bool is_disabled(int timer_id) const { return disabled.test(int(timer_id)); }

    bool is_expired(int timer_id, uint32_t now) {
        if (active.test(timer_id)) {
            if (ticks_diff(expire_at[timer_id], now) <= 0) {
                deactivate(timer_id);
                return true;
            }
            return false;
        }
        // not active but not disabled => expired
        return is_inactive(timer_id);
    };

    // A simple GC step that deactivates expired timers to reduce regular checks
    void cleanup(uint32_t now) {
        for (size_t i = 0; i < TIMER_COUNT; i++) {
            if (active.test(i)) { is_expired(i, now); } // Deactivate
        }
    };

    etl::optional<uint32_t> get_next_deadline(uint32_t now) const {
        etl::optional<uint32_t> result;

        for (size_t i = 0; i < TIMER_COUNT; i++) {
            if (active.test(i)) {
                if (!result ||
                    ticks_diff(expire_at[i], now) < ticks_diff(*result, now))
                {
                    result = expire_at[i];
                }
            }
        }

        return result;
    };
    etl::atomic<bool> timers_changed{false};

private:
    // After expiration, timer becomes deactivated, but not disabled, to
    // keep expire status.
    bool is_inactive(int timer_id) const {
        return !active.test(timer_id) && !disabled.test(timer_id);
    }

    void deactivate(int timer_id) {
        active.clear(timer_id);
        disabled.clear(timer_id);
        timers_changed.store(true);
    }

    // Timestamps compare with care about overflow
    int32_t ticks_diff(uint32_t expiration, uint32_t now) const {
        return static_cast<int32_t>(expiration - now);
    }

    etl::array<uint32_t, TIMER_COUNT> expire_at{};

    AtomicBits<TIMER_COUNT> active;
    AtomicBits<TIMER_COUNT> disabled;
};

} // namespace pd
