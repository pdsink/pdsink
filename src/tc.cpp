#include "common_macros.h"
#include "idriver.h"
#include "pd_conf.h"
#include "port.h"
#include "tc.h"

#include <etl/array.h>

namespace pd {

enum TC_State {
    TC_DETACHED,
    TC_DETECTING,
    TC_SINK_ATTACHED,
    TC_STATE_COUNT
};

namespace {
    constexpr auto tc_state_to_desc(int state) -> const char* {
        switch (state) {
            case TC_DETACHED: return "TC_DETACHED";
            case TC_DETECTING: return "TC_DETECTING";
            case TC_SINK_ATTACHED: return "TC_SINK_ATTACHED";
            default: return "Unknown TC state";
        }
    }
} // namespace

//
// Macros to quick-create common methods
//
#define ON_ENTER_STATE_DEFAULT \
auto on_enter_state() -> etl::fsm_state_id_t override { \
    get_fsm_context().log_state(); \
    return No_State_Change; \
}

#define ON_UNKNOWN_EVENT_DEFAULT \
auto on_event_unknown(__maybe_unused const etl::imessage& event) -> etl::fsm_state_id_t { \
    return No_State_Change; \
}



class TC_DETACHED_State : public etl::fsm_state<TC, TC_DETACHED_State, TC_DETACHED, MsgSysUpdate> {
public:
    ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tc = get_fsm_context();
        tc.log_state();

        tc.port.is_attached = false;
        tc.port.timers.stop(PD_TIMEOUT::TC_VBUS_DEBOUNCE);
        tc.tcpc.req_set_polarity(TCPC_POLARITY::NONE);
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgSysUpdate& event) -> etl::fsm_state_id_t {
        auto& tc = get_fsm_context();

        if (tc.tcpc.is_set_polarity_done()) { return No_State_Change; }

        auto vbus_ok = tc.tcpc.is_vbus_ok();
        if (!vbus_ok) {
            tc.port.timers.stop(PD_TIMEOUT::TC_VBUS_DEBOUNCE);
            return No_State_Change;
        }

        if (tc.port.timers.is_disabled(PD_TIMEOUT::TC_VBUS_DEBOUNCE)) {
            tc.port.timers.start(PD_TIMEOUT::TC_VBUS_DEBOUNCE);
            return No_State_Change;
        }

        if (tc.port.timers.is_expired(PD_TIMEOUT::TC_VBUS_DEBOUNCE)) {
            return TC_DETECTING;
        }

        return No_State_Change;
    }

    void on_exit_state() override {
        get_fsm_context().port.timers.stop(PD_TIMEOUT::TC_VBUS_DEBOUNCE);
    }
};


class TC_DETECTING_State : public etl::fsm_state<TC, TC_DETECTING_State, TC_DETECTING, MsgSysUpdate> {
public:
    ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tc = get_fsm_context();
        tc.log_state();

        tc.prev_cc1 = TCPC_CC_LEVEL::NONE;
        tc.prev_cc2 = TCPC_CC_LEVEL::NONE;
        tc.tcpc.req_scan_cc();
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgSysUpdate& event) -> etl::fsm_state_id_t {
        auto& tc = get_fsm_context();

        if (!tc.port.timers.is_disabled(PD_TIMEOUT::TC_CC_POLL)) {
            if (!tc.port.timers.is_expired(PD_TIMEOUT::TC_CC_POLL)) { return No_State_Change; }

            tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
            tc.tcpc.req_scan_cc();
        }

        if (tc.tcpc.is_scan_cc_done()) { return No_State_Change; }

        if (!tc.tcpc.is_vbus_ok()) { return TC_DETACHED; }

        auto cc1 = tc.tcpc.get_cc(TCPC_CC::CC1);
        auto cc2 = tc.tcpc.get_cc(TCPC_CC::CC2);

        // We may have different SRC attached, but only PD-capable is acceptable
        if ((cc1 != cc2) && (cc1 == tc.prev_cc1) && (cc2 == tc.prev_cc2))
        {
            tc.tcpc.req_set_polarity(cc1 > cc2 ? TCPC_POLARITY::CC1 : TCPC_POLARITY::CC2);
            return TC_SINK_ATTACHED;
        }

        tc.prev_cc1 = cc1;
        tc.prev_cc2 = cc2;
        tc.port.timers.start(PD_TIMEOUT::TC_CC_POLL);
        return No_State_Change;
    }

    void on_exit_state() override {
        auto& tc = get_fsm_context();
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
    }
};


class TC_SINK_ATTACHED_State : public etl::fsm_state<TC, TC_SINK_ATTACHED_State, TC_SINK_ATTACHED, MsgSysUpdate> {
public:
    ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tc = get_fsm_context();
        tc.log_state();

        tc.port.is_attached = true;
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgSysUpdate& event) -> etl::fsm_state_id_t {
        auto& tc = get_fsm_context();

        // TODO: Actually, we should check Safe0v. Check, if we should be more
        // strict here. In theory, active CC also could be used, but it has
        // lot of zeroes during BMC transfers to filter out.
        if (!tc.tcpc.is_vbus_ok()) {
            return TC_DETACHED;
        }
        return No_State_Change;
    }
};

TC::TC(Port& port, ITCPC& tcpc)
    : etl::fsm(0), port{port}, tcpc{tcpc}, tc_event_listener{*this}
{
    static etl::array<etl::ifsm_state*, TC_State::TC_STATE_COUNT> tc_state_list = {{
        new TC_DETACHED_State(),
        new TC_DETECTING_State(),
        new TC_SINK_ATTACHED_State()
    }};

    set_states(tc_state_list.data(), TC_State::TC_STATE_COUNT);
};

void TC::log_state() {
    TC_LOG("TC state => {}", tc_state_to_desc(get_state_id()));
}

void TC::setup() {
    port.msgbus.subscribe(tc_event_listener);
    start();
}

void TC_EventListener::on_receive(const MsgSysUpdate& msg) {
    tc.receive(msg);
}

void TC_EventListener::on_receive_unknown(const etl::imessage& msg) {
    TC_LOG("TC unknown message, id: {}", msg.get_message_id());
}

} // namespace pd