#include "pd_conf.h"
#include "tc.h"
#include "sink.h"

namespace pd {

enum TC_State {
    TC_DETACHED,
    TC_DETECTING,
    TC_SINK_ATTACHED,
    TC_STATE_COUNT
};

namespace {
    constexpr const char* const tc_state_names[] = {
        [TC_DETACHED] = "TC_DETACHED",
        [TC_DETECTING] = "TC_DETECTING",
        [TC_SINK_ATTACHED] = "TC_SINK_ATTACHED"
    };

    static_assert(sizeof(tc_state_names) / sizeof(tc_state_names[0]) == TC_STATE_COUNT, "TC state names array size mismatch");
}

//
// Macros to quick-create common methods
//
#define ON_ENTER_STATE_DEFAULT \
auto on_enter_state() -> etl::fsm_state_id_t override { \
    get_fsm_context().log_state(); \
    return No_State_Change; \
}

#define ON_UNKNOWN_EVENT_DEFAULT \
auto on_event_unknown(const etl::imessage& event) -> etl::fsm_state_id_t { \
    return No_State_Change; \
}



class TC_DETACHED_State : public etl::fsm_state<TC, TC_DETACHED_State, TC_DETACHED, MsgPdEvents> {
public:
    ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tc = get_fsm_context();
        tc.log_state();

        tc.sink.timers.start(PD_TIMEOUT::TC_CC_POLL);
        tc.prev_cc1 = TCPC_CC_LEVEL::NONE;
        tc.prev_cc2 = TCPC_CC_LEVEL::NONE;
        tc.tcpc.req_cc_both();
        return No_State_Change;
    }

    auto on_event(const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& tc = get_fsm_context();

        if (tc.tcpc.get_state().test(TCPC_FLAG::REQ_CC_BOTH)) {
            return No_State_Change;
        }

        if (!tc.sink.timers.is_expired(PD_TIMEOUT::TC_CC_POLL)) {
            return No_State_Change;
        }

        auto cc1 = tc.tcpc.get_cc(TCPC_CC::CC1);
        auto cc2 = tc.tcpc.get_cc(TCPC_CC::CC2);

        // If any of CC not zero => cable attached, go to detecting
        if (cc1 != TCPC_CC_LEVEL::NONE || cc2 != TCPC_CC_LEVEL::NONE) {
            return TC_DETECTING;
        }

        // Rearm timer and fetch
        tc.sink.timers.start(PD_TIMEOUT::TC_CC_POLL);
        tc.tcpc.req_cc_both();

        return No_State_Change;
    }
};


class TC_DETECTING_State : public etl::fsm_state<TC, TC_DETECTING_State, TC_DETECTING, MsgPdEvents> {
public:
    ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tc = get_fsm_context();
        tc.log_state();

        tc.prev_cc1 = tc.tcpc.get_cc(TCPC_CC::CC1);
        tc.prev_cc2 = tc.tcpc.get_cc(TCPC_CC::CC2);
        tc.tcpc.req_cc_both();
        tc.sink.timers.start(PD_TIMEOUT::TC_CC_POLL);
        tc.sink.timers.start(PD_TIMEOUT::TC_CC_DEBOUNCE);
        return No_State_Change;
    }

    auto on_event(const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& tc = get_fsm_context();

        if (tc.tcpc.get_state().test(TCPC_FLAG::REQ_CC_BOTH)) {
            return No_State_Change;
        }

        if (!tc.sink.timers.is_expired(PD_TIMEOUT::TC_CC_POLL)) {
            return No_State_Change;
        }

        auto cc1 = tc.tcpc.get_cc(TCPC_CC::CC1);
        auto cc2 = tc.tcpc.get_cc(TCPC_CC::CC2);

        // We may have different SRC attached, but only PD-capable is acceptable
        if (((cc1 == TCPC_CC_LEVEL::RP_3_0 && cc2 == TCPC_CC_LEVEL::NONE) ||
            (cc1 == TCPC_CC_LEVEL::NONE && cc2 == TCPC_CC_LEVEL::RP_3_0)) &&
            (cc1 == tc.prev_cc1 || cc2 == tc.prev_cc2))
        {
            // Probably, polarity can be set only once after debounce,
            // but do it in advance, for sure.
            tc.tcpc.set_polarity(cc1 == TCPC_CC_LEVEL::RP_3_0 ? TCPC_CC::CC1 : TCPC_CC::CC2);

            if (tc.sink.timers.is_expired(PD_TIMEOUT::TC_CC_DEBOUNCE)) {
                return TC_SINK_ATTACHED;
            }
        }
        else
        {
            // On CC mismatch - restart debounce timer
            tc.sink.timers.start(PD_TIMEOUT::TC_CC_DEBOUNCE);
        }

        // Not yet debounced => rearm polling
        tc.prev_cc1 = cc1;
        tc.prev_cc2 = cc2;
        tc.tcpc.req_cc_both();
        tc.sink.timers.start(PD_TIMEOUT::TC_CC_POLL);
        return No_State_Change;
    }

    void on_exit_state() override {
        auto& sink = get_fsm_context().sink;
        sink.timers.stop(PD_TIMEOUT::TC_CC_POLL);
        sink.timers.stop(PD_TIMEOUT::TC_CC_DEBOUNCE);
    }
};


class TC_SINK_ATTACHED_State : public etl::fsm_state<TC, TC_SINK_ATTACHED_State, TC_SINK_ATTACHED, MsgPdEvents> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_ENTER_STATE_DEFAULT;

    auto on_event(const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& tc = get_fsm_context();

        // If active CC become zero => cable detached
        // Note, value of active CC is updated by driver automatically. Here
        // we just quick-read cache, without boring chip request/debouncing.
        if (tc.tcpc.get_cc(TCPC_CC::ACTIVE) == TCPC_CC_LEVEL::NONE) {
            return TC_DETACHED;
        }
        return No_State_Change;
    }
};


TC::TC(Sink& sink, ITCPC& tcpc) : etl::fsm(0), sink{sink}, tcpc{tcpc} {
    sink.tc = this;

    static etl::ifsm_state* tc_state_list[TC_State::TC_STATE_COUNT] = {
        new TC_DETACHED_State(),
        new TC_DETECTING_State(),
        new TC_SINK_ATTACHED_State()
    };

    set_states(tc_state_list, TC_State::TC_STATE_COUNT);
};

void TC::log_state() {
    TC_LOG("TC state => %s", tc_state_names[get_state_id()]);
}

void TC::dispatch(const MsgPdEvents& events) {
    if (!events.has_timeout() && !(events.value & PD_EVENT::DRV_UPDATE)) {
        return;
    }

    receive(events);
}

bool TC::is_connected() {
    return is_started() && get_state_id() == TC_SINK_ATTACHED;
}

} // namespace pd