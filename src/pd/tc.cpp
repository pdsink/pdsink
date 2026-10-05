#include <etl/array.h>

#include "idriver.h"
#include "pd_log.h"
#include "port.h"
#include "tc.h"
#include "utils/etl_state_pack.h"

namespace pd {

using afsm::state_id_t;

enum TC_State {
    TC_UNATTACHED_SNK,
    TC_UNATTACHED_SNK_TOGGLING,
    TC_UNATTACHED_SNK_CC_DETECT,
    TC_ATTACH_WAIT_SNK_SET_POLARITY,
    TC_ATTACH_WAIT_SNK_CC_STABLE,
    TC_ATTACH_WAIT_SNK_CC_LOSS,
    TC_ATTACH_WAIT_SNK_VBUS_CHECK,
    TC_ATTACHED_SNK,
    TC_STATE_COUNT
};

namespace {
    constexpr auto tc_state_to_desc(int state) {
        switch (state) {
            case TC_UNATTACHED_SNK:
                return PD_LOG_ASSUME_STATIC_STR("TC_UNATTACHED_SNK");
            case TC_UNATTACHED_SNK_TOGGLING:
                return PD_LOG_ASSUME_STATIC_STR("TC_UNATTACHED_SNK_TOGGLING");
            case TC_UNATTACHED_SNK_CC_DETECT:
                return PD_LOG_ASSUME_STATIC_STR("TC_UNATTACHED_SNK_CC_DETECT");
            case TC_ATTACH_WAIT_SNK_SET_POLARITY:
                return PD_LOG_ASSUME_STATIC_STR("TC_ATTACH_WAIT_SNK_SET_POLARITY");
            case TC_ATTACH_WAIT_SNK_CC_STABLE:
                return PD_LOG_ASSUME_STATIC_STR("TC_ATTACH_WAIT_SNK_CC_STABLE");
            case TC_ATTACH_WAIT_SNK_CC_LOSS:
                return PD_LOG_ASSUME_STATIC_STR("TC_ATTACH_WAIT_SNK_CC_LOSS");
            case TC_ATTACH_WAIT_SNK_VBUS_CHECK:
                return PD_LOG_ASSUME_STATIC_STR("TC_ATTACH_WAIT_SNK_VBUS_CHECK");
            case TC_ATTACHED_SNK:
                return PD_LOG_ASSUME_STATIC_STR("TC_ATTACHED_SNK");
            default:
                return PD_LOG_ASSUME_STATIC_STR("Unknown TC state");
        }
    }

    bool is_cc_nonzero(TCPC_CC_LEVEL::Type level) {
        return level != TCPC_CC_LEVEL::NONE;
    }

    // Space out CC update requests; return true when data can be used.
    // In on_enter, stop TC_CC_POLL and call req_fetch_cc(ACTIVE_CC); call this in on_run.
    // Stop TC_CC_POLL in on_exit.
    bool poll_active_cc(TC& tc) {
        auto& timers = tc.port.timers;

        if (!timers.is_disabled(PD_TIMEOUT::TC_CC_POLL) &&
            timers.is_expired(PD_TIMEOUT::TC_CC_POLL))
        {
            timers.stop(PD_TIMEOUT::TC_CC_POLL);
            tc.tcpc.req_fetch_cc(TCPC_CC_REQ::ACTIVE_CC);
            return false;
        }

        if (!tc.tcpc.is_fetch_cc_done()) {
            return false;
        }

        if (timers.is_disabled(PD_TIMEOUT::TC_CC_POLL)) {
            timers.start(PD_TIMEOUT::TC_CC_POLL);
        }
        return true;
    }
} // namespace

class TC_UNATTACHED_SNK_State :
    public afsm::state<TC, TC_UNATTACHED_SNK_State, TC_UNATTACHED_SNK> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        auto& port = tc.port;
        tc.log_state();

        port.is_attached = false;
        port.notify_dpm(MsgToDpm_CableDetached{});
        port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
        port.timers.stop(PD_TIMEOUT::tCCDebounce);
        tc.detected_polarity = TCPC_POLARITY::NONE;
        tc.tcpc.req_set_polarity(TCPC_POLARITY::NONE);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!tc.tcpc.is_set_polarity_done()) { return No_State_Change; }
        return tc.tcpc.get_hw_features().toggling
            ? TC_UNATTACHED_SNK_TOGGLING
            : TC_UNATTACHED_SNK_CC_DETECT;
    }

    static void on_exit_state(TC&) {}
};

class TC_UNATTACHED_SNK_TOGGLING_State :
    public afsm::state<TC, TC_UNATTACHED_SNK_TOGGLING_State, TC_UNATTACHED_SNK_TOGGLING> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        tc.tcpc.req_set_polarity(TCPC_POLARITY::TOGGLING);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!tc.tcpc.is_set_polarity_done()) { return No_State_Change; }

        const auto polarity = tc.tcpc.get_polarity();
        if (polarity != TCPC_POLARITY::CC1 && polarity != TCPC_POLARITY::CC2) {
            return No_State_Change;
        }

        return TC_ATTACH_WAIT_SNK_CC_STABLE;
    }

    static void on_exit_state(TC&) {}
};

class TC_UNATTACHED_SNK_CC_DETECT_State :
    public afsm::state<TC, TC_UNATTACHED_SNK_CC_DETECT_State, TC_UNATTACHED_SNK_CC_DETECT> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
        tc.tcpc.req_fetch_cc(TCPC_CC_REQ::CC1CC2);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        auto& timers = tc.port.timers;

        if (!timers.is_disabled(PD_TIMEOUT::TC_CC_POLL)) {
            if (timers.is_expired(PD_TIMEOUT::TC_CC_POLL)) {
                timers.stop(PD_TIMEOUT::TC_CC_POLL);
                tc.tcpc.req_fetch_cc(TCPC_CC_REQ::CC1CC2);
            }
            return No_State_Change;
        }

        if (!tc.tcpc.is_fetch_cc_done()) { return No_State_Change; }

        const auto cc1 = tc.tcpc.get_cc(TCPC_CC_GET::CC1);
        const auto cc2 = tc.tcpc.get_cc(TCPC_CC_GET::CC2);

        const bool cc1_nonzero = is_cc_nonzero(cc1);
        const bool cc2_nonzero = is_cc_nonzero(cc2);
        if (cc1_nonzero != cc2_nonzero) {
            tc.detected_polarity = cc1_nonzero ? TCPC_POLARITY::CC1 : TCPC_POLARITY::CC2;
            return TC_ATTACH_WAIT_SNK_SET_POLARITY;
        }

        timers.start(PD_TIMEOUT::TC_CC_POLL);
        return No_State_Change;
    }

    static void on_exit_state(TC& tc) {
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
    }
};

class TC_ATTACH_WAIT_SNK_SET_POLARITY_State :
    public afsm::state<TC, TC_ATTACH_WAIT_SNK_SET_POLARITY_State, TC_ATTACH_WAIT_SNK_SET_POLARITY> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        tc.tcpc.req_set_polarity(tc.detected_polarity);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!tc.tcpc.is_set_polarity_done()) { return No_State_Change; }
        return TC_ATTACH_WAIT_SNK_CC_STABLE;
    }

    static void on_exit_state(TC&) {}
};

class TC_ATTACH_WAIT_SNK_CC_STABLE_State :
    public afsm::state<TC, TC_ATTACH_WAIT_SNK_CC_STABLE_State, TC_ATTACH_WAIT_SNK_CC_STABLE> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
        tc.port.timers.start(PD_TIMEOUT::tCCDebounce);
        tc.tcpc.req_fetch_cc(TCPC_CC_REQ::ACTIVE_CC);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!poll_active_cc(tc)) { return No_State_Change; }
        const auto cc_level = tc.tcpc.get_cc(TCPC_CC_GET::ACTIVE_CC);

        if (!is_cc_nonzero(cc_level)) { return TC_ATTACH_WAIT_SNK_CC_LOSS; }

        auto& timers = tc.port.timers;
        if (!timers.is_expired(PD_TIMEOUT::tCCDebounce)) {
            return No_State_Change;
        }

        return tc.tcpc.is_vbus_ok()
            ? TC_ATTACHED_SNK : TC_ATTACH_WAIT_SNK_VBUS_CHECK;
    }

    static void on_exit_state(TC& tc) {
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
        tc.port.timers.stop(PD_TIMEOUT::tCCDebounce);
    }
};

class TC_ATTACH_WAIT_SNK_CC_LOSS_State :
    public afsm::state<TC, TC_ATTACH_WAIT_SNK_CC_LOSS_State, TC_ATTACH_WAIT_SNK_CC_LOSS> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
        tc.port.timers.start(PD_TIMEOUT::tPDDebounce);
        tc.tcpc.req_fetch_cc(TCPC_CC_REQ::ACTIVE_CC);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!poll_active_cc(tc)) { return No_State_Change; }
        const auto cc_level = tc.tcpc.get_cc(TCPC_CC_GET::ACTIVE_CC);

        if (is_cc_nonzero(cc_level)) { return TC_ATTACH_WAIT_SNK_CC_STABLE; }
        if (tc.port.timers.is_expired(PD_TIMEOUT::tPDDebounce)) {
            return TC_UNATTACHED_SNK;
        }
        return No_State_Change;
    }

    static void on_exit_state(TC& tc) {
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
        tc.port.timers.stop(PD_TIMEOUT::tPDDebounce);
    }
};

class TC_ATTACH_WAIT_SNK_VBUS_CHECK_State :
    public afsm::state<TC, TC_ATTACH_WAIT_SNK_VBUS_CHECK_State, TC_ATTACH_WAIT_SNK_VBUS_CHECK> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
        tc.tcpc.req_fetch_cc(TCPC_CC_REQ::ACTIVE_CC);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!poll_active_cc(tc)) { return No_State_Change; }
        const auto cc_level = tc.tcpc.get_cc(TCPC_CC_GET::ACTIVE_CC);

        if (!is_cc_nonzero(cc_level)) { return TC_ATTACH_WAIT_SNK_CC_LOSS; }
        if (tc.tcpc.is_vbus_ok()) { return TC_ATTACHED_SNK; }
        return No_State_Change;
    }

    static void on_exit_state(TC& tc) {
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL);
    }
};

class TC_ATTACHED_SNK_State :
    public afsm::state<TC, TC_ATTACHED_SNK_State, TC_ATTACHED_SNK> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        if (!tc.tcpc.is_vbus_ok()) { return TC_UNATTACHED_SNK; }

        tc.port.is_attached = true;
        tc.port.notify_dpm(MsgToDpm_CableAttached{});
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!tc.tcpc.is_vbus_ok()) { return TC_UNATTACHED_SNK; }
        return No_State_Change;
    }

    static void on_exit_state(TC&) {}
};

using TC_STATES = afsm::state_pack<
    TC_UNATTACHED_SNK_State,
    TC_UNATTACHED_SNK_TOGGLING_State,
    TC_UNATTACHED_SNK_CC_DETECT_State,
    TC_ATTACH_WAIT_SNK_SET_POLARITY_State,
    TC_ATTACH_WAIT_SNK_CC_STABLE_State,
    TC_ATTACH_WAIT_SNK_CC_LOSS_State,
    TC_ATTACH_WAIT_SNK_VBUS_CHECK_State,
    TC_ATTACHED_SNK_State
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
    change_state(TC_UNATTACHED_SNK, true);
}

void TC_EventListener::on_receive(const MsgSysUpdate&) {
    tc.run();
}

void TC_EventListener::on_receive_unknown(ETL_MAYBE_UNUSED const etl::imessage& msg) {
    TC_LOGE("TC unknown message, ID: {}", msg.get_message_id());
}

} // namespace pd
