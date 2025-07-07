#include "timers.h"

namespace pd {

void Timers::start(int timer_id, uint32_t us_time) {
    active.set(timer_id);
    disabled.clear(timer_id);
    expire_at[timer_id] = get_time() + us_time;
    timers_changed.store(true);
}

void Timers::stop(int timer_id) {
    active.clear(timer_id);
    disabled.set(timer_id);
    timers_changed.store(true);
}

void Timers::stop_range(const PD_TIMEOUT::Type& range) {
    for (int i = range.first; i <= range.second; i++) {
        stop(i);
    }
    timers_changed.store(true);
}

auto Timers::is_expired(int timer_id) -> bool {
    if (active.test(timer_id)) {
        if (get_time() >= expire_at[timer_id]) {
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
    constexpr int32_t MAX_EXPIRE = 0x7FFFFFFF;

    auto now = get_time();
    int32_t min = MAX_EXPIRE;

    for (auto i = 0; i < PD_TIMER::PD_TIMER_COUNT; i++) {
        if (active.test(i)) {
            const uint64_t exp = expire_at[i];
            if (exp <= now) { return 0; }

            const auto diff = static_cast<int32_t>(exp - now);
            if (diff < min) { min = diff; }
        }
    }

    return min == MAX_EXPIRE ? NO_EXPIRE : min;
}

} // namespace pd
