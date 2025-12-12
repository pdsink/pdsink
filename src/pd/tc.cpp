#include <etl/array.h>

#include "idriver.h"
#include "pd_log.h"
#include "port.h"
#include "tc.h"
#include "utils/etl_state_pack.h"

namespace pd {

using afsm::state_id_t;

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


class TC_DETACHED_State : public afsm::state<TC, TC_DETACHED_State, TC_DETACHED> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        auto& port = tc.port;
        tc.log_state();

        port.is_attached = false;
        port.notify_dpm(MsgToDpm_CableDetached{});
        port.timers.stop(PD_TIMEOUT::TC_VBUS_DEBOUNCE);
        tc.tcpc.req_set_polarity(TCPC_POLARITY::NONE);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!tc.tcpc.is_set_polarity_done()) { return No_State_Change; }

        auto vbus_ok = tc.tcpc.is_vbus_ok();

        if (!vbus_ok) {
            if (!tc.port.timers.is_disabled(PD_TIMEOUT::TC_VBUS_DEBOUNCE)) {
                tc.port.timers.stop(PD_TIMEOUT::TC_VBUS_DEBOUNCE);
            }
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

    static void on_exit_state(TC& tc) {
        tc.port.timers.stop(PD_TIMEOUT::TC_VBUS_DEBOUNCE);
    }
};


class TC_DETECTING_State : public afsm::state<TC, TC_DETECTING_State, TC_DETECTING> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();

        tc.prev_cc1 = TCPC_CC_LEVEL::NONE;
        tc.prev_cc2 = TCPC_CC_LEVEL::NONE;
        tc.tcpc.req_scan_cc();
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        auto& port = tc.port;

        if (!port.timers.is_disabled(PD_TIMEOUT::TC_CC_POLL)) {
            if (!port.timers.is_expired(PD_TIMEOUT::TC_CC_POLL)) { return No_State_Change; }

            port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
            tc.tcpc.req_scan_cc();
        }

        TCPC_CC_LEVEL::Type cc1, cc2;
        if (!tc.tcpc.try_scan_cc_result(cc1, cc2)) { return No_State_Change; }

        if (!tc.tcpc.is_vbus_ok()) { return TC_DETACHED; }

        if ((cc1 != cc2) && (cc1 == tc.prev_cc1) && (cc2 == tc.prev_cc2))
        {
            tc.tcpc.req_set_polarity(cc1 > cc2 ? TCPC_POLARITY::CC1 : TCPC_POLARITY::CC2);
            return TC_SINK_ATTACHED;
        }

        tc.prev_cc1 = cc1;
        tc.prev_cc2 = cc2;
        port.timers.start(PD_TIMEOUT::TC_CC_POLL);
        return No_State_Change;
    }

    static void on_exit_state(TC& tc) {
        auto& port = tc.port;
        port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
    }
};


class TC_SINK_ATTACHED_State : public afsm::state<TC, TC_SINK_ATTACHED_State, TC_SINK_ATTACHED> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        auto& port = tc.port;

        // If just entered, wait for polarity set to complete and then set the attached status.
        if (!port.is_attached) {
            if (!tc.tcpc.is_set_polarity_done()) { return No_State_Change; }
            port.is_attached = true;
            port.notify_dpm(MsgToDpm_CableAttached{});
        }

        // TODO: Actually, we should check Safe0v. Check if we should be more
        // strict here. In theory, active CC could also be used, but it has
        // a lot of zeros during BMC transfers to filter out.
        if (!tc.tcpc.is_vbus_ok()) {
            return TC_DETACHED;
        }
        return No_State_Change;
    }

    static void on_exit_state(TC&) {}
};

using TC_STATES = afsm::state_pack<
    TC_DETACHED_State,
    TC_DETECTING_State,
    TC_SINK_ATTACHED_State
>;

TC::TC(Port& port, ITCPC& tcpc)
    : port{port}, tcpc{tcpc}, tc_event_listener{*this}
{
    set_states<TC_STATES>();
}

void TC::log_state() const {
    TC_LOGI("TC state => {}", tc_state_to_desc(get_state_id()));
}

void TC::setup() {
    port.tc_rtr = &tc_event_listener;
    change_state(TC_DETACHED, true);
}

void TC_EventListener::on_receive(const MsgSysUpdate&) {
    tc.run();
}

void TC_EventListener::on_receive_unknown(ETL_MAYBE_UNUSED const etl::imessage& msg) {
    TC_LOGE("TC unknown message, ID: {}", msg.get_message_id());
}

} // namespace pd
