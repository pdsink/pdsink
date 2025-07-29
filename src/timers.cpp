#include <etl/limits.h>

#include "timers.h"

namespace pd {

static_assert(etl::is_same<PD_TIME_T, uint32_t>::value ||
              etl::is_same<PD_TIME_T, uint64_t>::value,
              "PD_TIME_T must be uint32_t or uint64_t");

void Timers::start(int timer_id, uint32_t time) {
    active.set(timer_id);
    disabled.clear(timer_id);
    expire_at[timer_id] = get_time() + time;
    timers_changed.store(true);
}

void Timers::stop(int timer_id) {
    active.clear(timer_id);
    disabled.set(timer_id);
    timers_changed.store(true);
}

void Timers::stop_range(const PD_TIMEOUT::Type& range) {
    for (uint32_t i = range.first; i <= range.second; i++) {
        stop(i);
    }
    timers_changed.store(true);
}

auto Timers::is_expired(int timer_id) -> bool {
    if (active.test(timer_id)) {
        if (time_diff(expire_at[timer_id], get_time()) <= 0) {
            deactivate(timer_id);
            return true;
        }
        return false;
    }
    // not active but not disabled => expired
    return is_inactive(timer_id);
};

void Timers::cleanup() {
    for (int i = 0; i < PD_TIMER::PD_TIMER_COUNT; i++) {
        if (active.test(i) && is_expired(i)) {
            deactivate(i);
        }
    }
}

auto Timers::get_next_expiration() -> int32_t {
    constexpr int32_t MAX_EXPIRE = etl::numeric_limits<int32_t>::max();

    auto now = get_time();
    int32_t min = MAX_EXPIRE;

    for (auto i = 0; i < PD_TIMER::PD_TIMER_COUNT; i++) {
        if (active.test(i)) {
            auto exp_diff = time_diff(expire_at[i], now);
            if (exp_diff <= 0) { return 0; }
            if (exp_diff < min) { min = exp_diff; }
        }
    }

    return min == MAX_EXPIRE ? NO_EXPIRE : min;
}

} // namespace pd
