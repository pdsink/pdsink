#pragma once

#include "utils/atomic_bits.h"

#include <etl/array.h>
#include <etl/atomic.h>
#include <etl/limits.h>

namespace pd {

template<size_t TIMER_COUNT>
class TimerPack {
public:
    explicit TimerPack() {
        active.clear_all();
        disabled.set_all();
        timers_changed.store(false);
    }

    void set_time(uint32_t time) {
        now = time;
    }

    void start(int timer_id, uint32_t period) {
        active.set(timer_id);
        disabled.clear(timer_id);
        expire_at[timer_id] = now + period;
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

    bool is_expired(int timer_id) {
        if (active.test(timer_id)) {
            if (time_diff(expire_at[timer_id], now) <= 0) {
                deactivate(timer_id);
                return true;
            }
            return false;
        }
        // not active but not disabled => expired
        return is_inactive(timer_id);
    };

    // A kind of gc - deactivates expired timers to reduce regular checks
    void cleanup() {
        for (size_t i = 0; i < TIMER_COUNT; i++) {
            if (active.test(i)) { is_expired(i); } // Deactivate
        }
    };

    // May be used for precise timer management. If regular 1ms interrupts are
    // used, that's not needed.
    int32_t get_next_expiration() const {
        constexpr int32_t MAX_EXPIRE = etl::numeric_limits<int32_t>::max();

        int32_t min = MAX_EXPIRE;

        for (size_t i = 0; i < TIMER_COUNT; i++) {
            if (active.test(i)) {
                auto exp_diff = time_diff(expire_at[i], now);
                if (exp_diff <= 0) { return 0; }
                if (exp_diff < min) { min = exp_diff; }
            }
        }

        return min == MAX_EXPIRE ? NO_EXPIRE : min;
    };

    static constexpr int32_t NO_EXPIRE = -1;
    etl::atomic<bool> timers_changed{false};

private:
    uint32_t now{0};

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
    int32_t time_diff(uint32_t expiration, uint32_t now) const {
        return static_cast<int32_t>(expiration - now);
    }

    etl::array<uint32_t, TIMER_COUNT> expire_at{};

    AtomicBits<TIMER_COUNT> active;
    AtomicBits<TIMER_COUNT> disabled;
};

} // namespace pd
