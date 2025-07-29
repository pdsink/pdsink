#pragma once

#include "pd_conf.h"
#include "utils/atomic_bits.h"

#include <etl/array.h>
#include <etl/atomic.h>
#include <etl/delegate.h>
#include <etl/utility.h>

namespace pd {

#if PD_TIMER_RESOLUTION_US != 0
    static constexpr int ms_mult = 1000;
#else
    static constexpr int ms_mult = 1;
#endif

// Virtual Timer IDs
namespace PD_TIMER {
    enum Type {
        TC_DEBOUNCE,

        // (!) Check PD_TIMERS_RANGE after update
        PE_SinkWaitCapTimer,
        PE_SenderResponseTimer,
        PE_SinkRequestTimer,
        PE_PSTransitionTimer,
        PE_SinkPPSPeriodicTimer,
        PE_SinkEPRKeepAliveTimer,
        PE_SinkEPREnterTimer,
        PE_BISTContModeTimer,

        // (!) Check PD_TIMERS_RANGE after update
        PRL_HardResetCompleteTimer,
        PRL_ActiveCcPollingDebounce, // Custom, not PD spec
        // PRL_CRCReceive, // All hardware now supports auto GoodCRC
        PRL_ChunkSenderResponse,
        PRL_ChunkSenderRequest,

        PD_TIMER_COUNT
    };
} // namespace PD_TIMER

// Constants for bulk timers reset
struct PD_TIMERS_RANGE {
    using Type = etl::pair<PD_TIMER::Type, PD_TIMER::Type>;

    static constexpr Type PE{PD_TIMER::PE_SinkWaitCapTimer, PD_TIMER::PE_BISTContModeTimer};
    static constexpr Type PRL{PD_TIMER::PRL_HardResetCompleteTimer, PD_TIMER::PRL_ChunkSenderRequest};
};

// 6.6.22 Time Values and Timers
// {Timer ID, Timeout in us}
//
// Some timeouts can use the same timer. PD components operate with PD_TIMEOUT-s
// to hide those details.
struct PD_TIMEOUT {
    using Type = etl::pair<PD_TIMER::Type, uint32_t>;

    static constexpr Type TC_VBUS_DEBOUNCE {PD_TIMER::TC_DEBOUNCE, 100 * ms_mult}; // 100-200 ms
    static constexpr Type TC_CC_POLL {PD_TIMER::TC_DEBOUNCE, 20 * ms_mult}; // 100-200 ms

    static constexpr Type tTypeCSinkWaitCap {PD_TIMER::PE_SinkWaitCapTimer, 465 * ms_mult}; // 310-620 ms
    static constexpr Type tSenderResponse {PD_TIMER::PE_SenderResponseTimer, 30 * ms_mult}; // 27-36 ms
    static constexpr Type tSinkRequest {PD_TIMER::PE_SinkRequestTimer, 100 * ms_mult}; // 100 ms before repeat
    static constexpr Type tPPSRequest {PD_TIMER::PE_SinkPPSPeriodicTimer, 5000 * ms_mult}; // 10s max
    // PS Transition timeout depends on mode
    static constexpr Type tPSTransition_SPR {PD_TIMER::PE_PSTransitionTimer, 500 * ms_mult}; // 450-550 ms
    static constexpr Type tPSTransition_EPR {PD_TIMER::PE_PSTransitionTimer, 925 * ms_mult}; // 830-1020 ms
    static constexpr Type tSinkEPRKeepAlive {PD_TIMER::PE_SinkEPRKeepAliveTimer, 375 * ms_mult}; // 250-500 ms
    static constexpr Type tEnterEPR {PD_TIMER::PE_SinkEPREnterTimer, 500 * ms_mult}; // 450-550 ms
    static constexpr Type tBISTCarrierMode {PD_TIMER::PE_BISTContModeTimer, 300 * ms_mult}; // 300 ms before exit

    static constexpr Type tHardResetComplete {PD_TIMER::PRL_HardResetCompleteTimer, 5 * ms_mult}; // 4-5 ms
    static constexpr Type tChunkSenderResponse {PD_TIMER::PRL_ChunkSenderResponse, 27 * ms_mult}; // 24-30 ms
    static constexpr Type tChunkSenderRequest {PD_TIMER::PRL_ChunkSenderRequest, 27 * ms_mult}; // 24-30 ms

    // This should be 1.0ms, but if timer precision is 1ms only,
    // we use 2ms to be sure AT LEAST 1ms passed. Anyway, that's not critical,
    // because all modern TCPC process GoodCRC in hardware. Probably we should
    // remove that logic at all, when drivers architecture become more clear.
    // #if PD_TIMER_RESOLUTION_US != 0
    //     static constexpr Type tReceive {PD_TIMER::PRL_CRCReceive, 1 * ms_mult}; // 0.9-1.1 ms
    // #else
    //     static constexpr Type tReceive {PD_TIMER::PRL_CRCReceive, 2 * ms_mult}; // 0.9-1.1 ms
    // #endif

    // CC polling timeout for waiting SnkTxOK level before AMS transfer.
    static constexpr Type tActiveCcPollingDebounce {PD_TIMER::PRL_ActiveCcPollingDebounce, 20 * ms_mult}; // 20 ms
};

class Timers {
public:
    using GetTimeFunc = etl::delegate<PD_TIME_T()>;

    explicit Timers() {
        active.clear_all();
        disabled.set_all();
        timers_changed.store(false);
    }

    void set_time_provider(const GetTimeFunc& func) {
        get_time_func = func;
    }

    void start(int timer_id, uint32_t time);
    void stop(int timer_id);
    void stop_range(const PD_TIMEOUT::Type& range);
    inline bool is_disabled(int timer_id) { return disabled.test(int(timer_id)); }
    bool is_expired(int timer_id);

    // Duplicated methods, accepting PD_TIMEOUT::Type, for convenience
    inline void start(const PD_TIMEOUT::Type& timeout) {
        start(timeout.first, timeout.second);
    }
    inline void stop(const PD_TIMEOUT::Type& timeout) {
        stop(timeout.first);
    }
    inline bool is_disabled(const PD_TIMEOUT::Type& timeout) {
        return is_disabled(timeout.first);
    }
    inline bool is_expired(const PD_TIMEOUT::Type& timeout) {
        return is_expired(timeout.first);
    }

    // A kind of gc - deactivates expired timers to reduce regular checks
    void cleanup();

    // May be used for precise timer management. If regular 1ms interrupts
    // used, that's not needed.
    int32_t get_next_expiration();

    static constexpr int32_t NO_EXPIRE = -1;
    etl::atomic<bool> timers_changed{false};

private:
    GetTimeFunc get_time_func;

    PD_TIME_T get_time() {
        return get_time_func ? get_time_func() : 0;
    }

    // After expiration, timer becomes deactivated, but not disabled, to
    // keep expire status.
    inline bool is_inactive(int timer_id) {
        return !active.test(timer_id) && !disabled.test(timer_id);
    }
    inline void deactivate(int timer_id) {
        active.clear(timer_id);
        disabled.clear(timer_id);
    }

    // Timestamps compare with care about overflow
    int32_t time_diff(PD_TIME_T expiration, PD_TIME_T now) {
        return static_cast<int32_t>(expiration - now);
    }

    etl::array<PD_TIME_T, PD_TIMER::PD_TIMER_COUNT> expire_at{};

    AtomicBits<PD_TIMER::PD_TIMER_COUNT> active;
    AtomicBits<PD_TIMER::PD_TIMER_COUNT> disabled;
};


} // namespace pd