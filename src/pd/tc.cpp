#include <etl/array.h>

#include "idriver.h"
#include "pd_log.h"
#include "port.h"
#include "tc.h"
#include "utils/etl_state_pack.h"
#include "utils/state_names.h"

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
    PD_DEFINE_STATE_NAMES(tc_state_to_desc, "Unknown TC state",
        PD_STATE_NAME(TC_UNATTACHED_SNK),
        PD_STATE_NAME(TC_UNATTACHED_SNK_TOGGLING),
        PD_STATE_NAME(TC_UNATTACHED_SNK_CC_DETECT),
        PD_STATE_NAME(TC_ATTACH_WAIT_SNK_SET_POLARITY),
        PD_STATE_NAME(TC_ATTACH_WAIT_SNK_CC_STABLE),
        PD_STATE_NAME(TC_ATTACH_WAIT_SNK_CC_LOSS),
        PD_STATE_NAME(TC_ATTACH_WAIT_SNK_VBUS_CHECK),
        PD_STATE_NAME(TC_ATTACHED_SNK)
    );

    bool is_cc_nonzero(TCPC_CC_LEVEL::Type level) {
        return level != TCPC_CC_LEVEL::NONE;
    }

    // Space out CC update requests; return true when data can be used.
    // In on_enter, stop TC_CC_POLL_PERIOD and call req_fetch_cc(ACTIVE_CC); call this in on_run.
    // Stop TC_CC_POLL_PERIOD in on_exit.
    bool poll_active_cc(TC& tc) {
        auto& timers = tc.port.timers;

        if (!timers.is_disabled(PD_TIMEOUT::TC_CC_POLL_PERIOD) &&
            timers.is_expired(PD_TIMEOUT::TC_CC_POLL_PERIOD))
        {
            timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
            tc.port.driver->req_fetch_cc(TCPC_CC_REQ::ACTIVE_CC);
            return false;
        }

        if (!tc.port.driver->is_fetch_cc_done()) {
            return false;
        }

        if (timers.is_disabled(PD_TIMEOUT::TC_CC_POLL_PERIOD)) {
            timers.start(PD_TIMEOUT::TC_CC_POLL_PERIOD);
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
        port.timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
        port.timers.stop(PD_TIMEOUT::tCCDebounce);
        tc.detected_polarity = TCPC_POLARITY::NONE;
        tc.port.driver->req_set_polarity(TCPC_POLARITY::NONE);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!tc.port.driver->is_set_polarity_done()) { return No_State_Change; }
        return tc.port.driver->get_features().cc_toggling
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
        tc.port.driver->req_set_polarity(TCPC_POLARITY::TOGGLING);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!tc.port.driver->is_set_polarity_done()) { return No_State_Change; }

        const auto polarity = tc.port.driver->get_polarity();
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
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
        tc.port.driver->req_fetch_cc(TCPC_CC_REQ::CC1CC2);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        auto& timers = tc.port.timers;

        if (!timers.is_disabled(PD_TIMEOUT::TC_CC_POLL_PERIOD)) {
            if (timers.is_expired(PD_TIMEOUT::TC_CC_POLL_PERIOD)) {
                timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
                tc.port.driver->req_fetch_cc(TCPC_CC_REQ::CC1CC2);
            }
            return No_State_Change;
        }

        if (!tc.port.driver->is_fetch_cc_done()) { return No_State_Change; }

        const auto cc1 = tc.port.driver->get_cc(TCPC_CC_GET::CC1);
        const auto cc2 = tc.port.driver->get_cc(TCPC_CC_GET::CC2);

        const bool cc1_nonzero = is_cc_nonzero(cc1);
        const bool cc2_nonzero = is_cc_nonzero(cc2);
        if (cc1_nonzero != cc2_nonzero) {
            tc.detected_polarity = cc1_nonzero ? TCPC_POLARITY::CC1 : TCPC_POLARITY::CC2;
            return TC_ATTACH_WAIT_SNK_SET_POLARITY;
        }

        timers.start(PD_TIMEOUT::TC_CC_POLL_PERIOD);
        return No_State_Change;
    }

    static void on_exit_state(TC& tc) {
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
    }
};

class TC_ATTACH_WAIT_SNK_SET_POLARITY_State :
    public afsm::state<TC, TC_ATTACH_WAIT_SNK_SET_POLARITY_State, TC_ATTACH_WAIT_SNK_SET_POLARITY> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        tc.port.driver->req_set_polarity(tc.detected_polarity);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!tc.port.driver->is_set_polarity_done()) { return No_State_Change; }
        return TC_ATTACH_WAIT_SNK_CC_STABLE;
    }

    static void on_exit_state(TC&) {}
};

class TC_ATTACH_WAIT_SNK_CC_STABLE_State :
    public afsm::state<TC, TC_ATTACH_WAIT_SNK_CC_STABLE_State, TC_ATTACH_WAIT_SNK_CC_STABLE> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
        tc.port.timers.start(PD_TIMEOUT::tCCDebounce);
        tc.port.driver->req_fetch_cc(TCPC_CC_REQ::ACTIVE_CC);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!poll_active_cc(tc)) { return No_State_Change; }
        const auto cc_level = tc.port.driver->get_cc(TCPC_CC_GET::ACTIVE_CC);

        if (!is_cc_nonzero(cc_level)) { return TC_ATTACH_WAIT_SNK_CC_LOSS; }

        auto& timers = tc.port.timers;
        if (!timers.is_expired(PD_TIMEOUT::tCCDebounce)) {
            return No_State_Change;
        }

        return tc.port.driver->check_vbus(TCPC_VBUS_LEVEL::PRESENT)
            ? TC_ATTACHED_SNK : TC_ATTACH_WAIT_SNK_VBUS_CHECK;
    }

    static void on_exit_state(TC& tc) {
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
        tc.port.timers.stop(PD_TIMEOUT::tCCDebounce);
    }
};

class TC_ATTACH_WAIT_SNK_CC_LOSS_State :
    public afsm::state<TC, TC_ATTACH_WAIT_SNK_CC_LOSS_State, TC_ATTACH_WAIT_SNK_CC_LOSS> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
        tc.port.timers.start(PD_TIMEOUT::tPDDebounce);
        tc.port.driver->req_fetch_cc(TCPC_CC_REQ::ACTIVE_CC);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!poll_active_cc(tc)) { return No_State_Change; }
        const auto cc_level = tc.port.driver->get_cc(TCPC_CC_GET::ACTIVE_CC);

        if (is_cc_nonzero(cc_level)) { return TC_ATTACH_WAIT_SNK_CC_STABLE; }
        if (tc.port.timers.is_expired(PD_TIMEOUT::tPDDebounce)) {
            return TC_UNATTACHED_SNK;
        }
        return No_State_Change;
    }

    static void on_exit_state(TC& tc) {
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
        tc.port.timers.stop(PD_TIMEOUT::tPDDebounce);
    }
};

class TC_ATTACH_WAIT_SNK_VBUS_CHECK_State :
    public afsm::state<TC, TC_ATTACH_WAIT_SNK_VBUS_CHECK_State, TC_ATTACH_WAIT_SNK_VBUS_CHECK> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
        tc.port.driver->req_fetch_cc(TCPC_CC_REQ::ACTIVE_CC);
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (!poll_active_cc(tc)) { return No_State_Change; }
        const auto cc_level = tc.port.driver->get_cc(TCPC_CC_GET::ACTIVE_CC);

        if (!is_cc_nonzero(cc_level)) { return TC_ATTACH_WAIT_SNK_CC_LOSS; }
        if (tc.port.driver->check_vbus(TCPC_VBUS_LEVEL::PRESENT)) { return TC_ATTACHED_SNK; }
        return No_State_Change;
    }

    static void on_exit_state(TC& tc) {
        tc.port.timers.stop(PD_TIMEOUT::TC_CC_POLL_PERIOD);
    }
};

class TC_ATTACHED_SNK_State :
    public afsm::state<TC, TC_ATTACHED_SNK_State, TC_ATTACHED_SNK> {
public:
    static auto on_enter_state(TC& tc) -> state_id_t {
        tc.log_state();
        if (tc.port.driver->check_vbus(TCPC_VBUS_LEVEL::SINK_DISCONNECTED)) { return TC_UNATTACHED_SNK; }

        tc.port.is_attached = true;
        tc.port.notify_dpm(MsgToDpm_CableAttached{});
        return No_State_Change;
    }

    static auto on_run_state(TC& tc) -> state_id_t {
        if (tc.port.driver->check_vbus(TCPC_VBUS_LEVEL::SINK_DISCONNECTED)) { return TC_UNATTACHED_SNK; }
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

TC::TC(Port& port)
    : port{port}, tc_event_listener{*this}
{
    set_states<TC_STATES>();
    port.tc_rtr = &tc_event_listener;
}

void TC::log_state() const {
    TC_LOGI("TC state => {}", tc_state_to_desc(get_state_id()));
}

void TC_EventListener::on_receive(const MsgSysUpdate&) {
    if (!tc.port.is_enabled.load()) {
        tc.change_state(afsm::Uninitialized);
        return;
    }

    if (tc.is_uninitialized()) {
        tc.change_state(TC_UNATTACHED_SNK);
    }

    tc.run();
}

void TC_EventListener::on_receive_unknown(ETL_MAYBE_UNUSED const etl::imessage& msg) {
    TC_LOGE("TC unknown message, ID: {}", msg.get_message_id());
}

} // namespace pd
