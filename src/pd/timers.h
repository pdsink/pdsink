#pragma once

#include <etl/utility.h>

#include "idriver.h"
#include "pd_conf.h"
#include "utils/timer_pack.h"

namespace pd {

// Virtual Timer IDs
namespace PD_TIMER {
    enum Type {
        TC_DEBOUNCE,
        TC_POLL,

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
        PRL_CRCReceive,
        PRL_ChunkSenderResponse,
        PRL_ChunkSenderRequest,

        PD_TIMER_COUNT
    };
} // namespace PD_TIMER

// Constants for bulk timers reset
struct PD_TIMERS_RANGE {
    using Type = etl::pair<int, int>;

    static constexpr Type PE{PD_TIMER::PE_SinkWaitCapTimer, PD_TIMER::PE_BISTContModeTimer};
    static constexpr Type PRL{PD_TIMER::PRL_HardResetCompleteTimer, PD_TIMER::PRL_ChunkSenderRequest};
};

// [rev3.2 v1.2] 7.31.19 Time Values and Timers
// {Timer ID, Timeout in us}
//
// Some timeouts can reuse the same timer. PD components operate with
// PD_TIMEOUT values to hide those details.
struct PD_TIMEOUT {
private:
    static constexpr uint32_t ms = 1000;

public:
    using Type = etl::pair<int, uint32_t>;

    // [USB Type-C 2.5] Table 4-34 and 4-3.
    static constexpr Type tCCDebounce {PD_TIMER::TC_DEBOUNCE, 100 * ms}; // 100-200 ms
    static constexpr Type tPDDebounce {PD_TIMER::TC_DEBOUNCE, 10 * ms}; // 10-20 ms
    static constexpr Type TC_CC_POLL {PD_TIMER::TC_POLL, 1 * ms};

    static constexpr Type tTypeCSinkWaitCap {PD_TIMER::PE_SinkWaitCapTimer, 465 * ms}; // 310-620 ms
    static constexpr Type tSenderResponse {PD_TIMER::PE_SenderResponseTimer, 30 * ms}; // 27-50 ms
    static constexpr Type tSinkRequest {PD_TIMER::PE_SinkRequestTimer, 100 * ms}; // 100 ms before repeat
    // 10s max in [rev3.2 v1.1] Table 6.68; [rev3.2 v1.2] Table 7.9 omits this value.
    static constexpr Type tPPSRequest {PD_TIMER::PE_SinkPPSPeriodicTimer, 5000 * ms};
    // PS Transition timeout depends on mode
    static constexpr Type tPSTransition_SPR {PD_TIMER::PE_PSTransitionTimer, 500 * ms}; // 450-550 ms
    static constexpr Type tPSTransition_EPR {PD_TIMER::PE_PSTransitionTimer, 925 * ms}; // 830-1020 ms
    static constexpr Type tSinkEPRKeepAlive {PD_TIMER::PE_SinkEPRKeepAliveTimer, 375 * ms}; // 250-500 ms
    static constexpr Type tEnterEPR {PD_TIMER::PE_SinkEPREnterTimer, 500 * ms}; // 450-550 ms
    static constexpr Type tBISTContMode {PD_TIMER::PE_BISTContModeTimer, 45 * ms}; // 30-60 ms

    static constexpr Type tHardResetComplete {PD_TIMER::PRL_HardResetCompleteTimer, 5 * ms}; // 4-5 ms
    static constexpr Type tChunkSenderResponse {PD_TIMER::PRL_ChunkSenderResponse, 27 * ms}; // 24-30 ms
    static constexpr Type tChunkSenderRequest {PD_TIMER::PRL_ChunkSenderRequest, 27 * ms}; // 24-30 ms

    static constexpr Type tReceive {PD_TIMER::PRL_CRCReceive, 1 * ms}; // 0.9-1.1 ms

    // CC polling timeout while waiting for the SnkTxOK level before AMS transfer.
    static constexpr Type tActiveCcPollingDebounce {PD_TIMER::PRL_ActiveCcPollingDebounce, 20 * ms}; // 20 ms
};

template<size_t TIMER_COUNT>
class Timers : public TimerPack<TIMER_COUNT> {
public:
    using Base = TimerPack<TIMER_COUNT>;

    void set_tick_source(ITimer::TickSource source) {
        tick_source = source;
    }

    void stop_range(const PD_TIMERS_RANGE::Type& range) {
        Base::stop_range(range.first, range.second);
    }

    void start(const PD_TIMEOUT::Type& timeout) {
        Base::start(timeout.first, get_ticks() + timeout.second);
    }

    void start_with_deadline(int timer_id, uint32_t deadline) {
        Base::start(timer_id, deadline);
    }

    void stop(const PD_TIMEOUT::Type& timeout) {
        Base::stop(timeout.first);
    }

    bool is_disabled(const PD_TIMEOUT::Type& timeout) {
        return Base::is_disabled(timeout.first);
    }

    bool is_expired(const PD_TIMEOUT::Type& timeout) {
        return Base::is_expired(timeout.first, get_ticks());
    }

    void cleanup() {
        Base::cleanup(get_ticks());
    }

    etl::optional<uint32_t> get_next_deadline() const {
        return Base::get_next_deadline(get_ticks());
    }

    uint32_t get_ticks() const {
        return tick_source ? tick_source() : 0;
    }

private:
    ITimer::TickSource tick_source{};
};

} // namespace pd
